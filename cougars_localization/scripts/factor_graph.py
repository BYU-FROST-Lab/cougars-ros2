#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty
from std_srvs.srv import SetBool
from scipy.spatial.transform import Rotation as R
import numpy as np
from functools import partial
import gtsam
from typing import List, Optional
from gtsam.symbol_shorthand import L

from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
# from factor_plot import Plotter
# from factor_class_plot import Plotter as Plot
# from factor_class_plot import Series


class Agent():
    def __init__(self, H_init):
        self.pose_world = gtsam.Pose3(H_init)
        self.poseKey = int(1)
        self.prevPoseKey = self.poseKey

DUMMY_DEPTH_VAL = 5.0
DUMMY_IMU_VAL = 10.0
DEPTH_THRESHOLD = -0.25

class FactorGraphNode(Node):

    def __init__(self):
        super().__init__('factor_graph_node')
        #PLOT
        # self.plot = Plotter()

        # Create Plotter objects with different series
        # self.x_output = Series('Output', 'b', size=400)
        # self.x_dvl = Series('DVL', 'r', alpha=0.5)
        # delta_series_2 = Series(name='Delta 2', color='green', size=600, alpha=0.5)
        

        self.prev_x = 0

        # number of seconds we take a dvl dead reck. pose/factor graph callback time
        self.declare_parameter("factor_callback_seconds",.8) #was 2s
        self.dvl_time_interval =  self.get_parameter("factor_callback_seconds").get_parameter_value().integer_value 
        self.declare_parameter("using_imu",True)
        self.declare_parameter("using_depth",True)
        self.declare_parameter("using_gps",True)
        self.declare_parameter("max_factor_graph_size",65) #max number of factors

        # flags to start the whole system
        self.depth_received = False
        self.gps_received = False
        self.imu_received = False
        self.dvl_received = False

        # measurement queues for sensors
        self.q_depth = []
        self.q_imu = []
        self.q_gps = []
        self.q_dvl = []

        # map of poseKeys to time stamps - dvl synced
        self.poseKey_to_time = {}
        
        # 'pointer' to mark the oldest pose that we shouldn't worry about adding a unary factor to
        # for example, if we add a gps unary factor to the second pose, than the gps_last_pose_key 
        # will point to the second pose. Or if gps goes out for a bit, this should point to the oldest
        # pose that we will not consider adding a unary factor to
        self.gps_last_pose_key = None       
        self.depth_last_pose_key = None
        self.imu_last_pose_key = None   

        # gtsam stuff

        # noise models
        self.std_pose = np.array([0.001, 0.001, 0.001, np.deg2rad(0.5), np.deg2rad(0.5), np.deg2rad(0.5)])
        self.DVL_NOISE = gtsam.noiseModel.Diagonal.Sigmas(self.std_pose)
        self.GPS_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([10.0, 12.0, 30.0])) #values based off of a few bags' fix messages, can be changed
        self.std_orientation = np.deg2rad(3)
        self.UNARY_HEADING_NOISE = gtsam.noiseModel.Isotropic.Sigma(3, self.std_orientation)
        self.std_depth = 0.1
        self.DEPTH_NOISE = gtsam.noiseModel.Isotropic.Sigma(1,self.std_depth )

        # incremental smoothing and mapping (isam) optimizer
        self.isam = gtsam.ISAM2()
        # factor graph
        self.graph = gtsam.NonlinearFactorGraph()
        # factor graph estimate
        self.initialEstimate = gtsam.Values()

    
        # Initialize class variables
        self.orientation_matrix = np.eye(3)
        self.orientation_covariance = np.zeros((3, 3))
        self.position = np.zeros(3)
        self.position_covariance = np.zeros((3, 3))
        self.dvl_position = np.zeros(3)
        self.dvl_position_covariance = np.zeros((3, 3))
        self.init_state = {}

        # flag to indicate that we have added a prior factor to pose 1
        # we now will begin the timer to add between factors and unary factors
        self.deployed = False

        # Subscribers

        # sensor subscriptions
        self.create_subscription(Imu, 'modem_imu', self.imu_callback, 10) # for unary factor
        self.create_subscription(PoseWithCovarianceStamped, 'depth_data', self.depth_callback, 10) # for unary factor
        self.create_subscription(Odometry, 'gps_odom', self.gps_callback, 10) # for unary factor
        self.create_subscription(PoseWithCovarianceStamped, 'dvl/dead_reckoning', self.dvl_callback, 10) # for between factor (dead reckon. pose to pose)
        # signal to add prior (should be sent after DVL-lock and ref. gps/ heading is stored and DVL is restarted)

        # Service
        # self.create_subscription(Empty, 'init', self.init_callback, 10)
        self.create_service(SetBool, 'init_factor_graph', self.init_callback)

        # Publisher
        # publishes the LATEST output of the smoothing and mapping (most importantly gps x,y), although remember the whole path is being smoothed
        self.vehicle_status_pub = self.create_publisher(Odometry, 'smoothed_output', 10)
        self.odom_msg = Odometry()
        
        # Timer for adding state estimate/factors
        self.timer = self.create_timer(self.dvl_time_interval, self.factor_graph_timer)
        
        # Initialize the transform broadcaster
        self.tf_broadcaster = StaticTransformBroadcaster(self)
    
    ###############################################
    ##### error functions for unary factors #######
    ###############################################

    def get_unary_heading(self, pose, measurement):

        rot_pred = pose.rotation()

        # Get full relative pose meas
        rot_meas = measurement[0]

        # print("rot pred", rot_pred)
        # print("rot meas", rot_meas)

        error = gtsam.Rot3.Logmap(rot_pred.between(rot_meas))
        # print("between anchor error", error)

        return error

    def error_unary_heading(self, measurement: np.ndarray, this: gtsam.CustomFactor,
                values: gtsam.Values,
                jacobians: Optional[List[np.ndarray]]) -> float:
        
        key_pose = this.keys()[0]
        pose = values.atPose3(key_pose)

        if jacobians is not None:
            eps = 1e-6

            # Six rows bc error state, six columns bc the state of this node is Pose3
            # Perturb relative to robot A pose key
            H0 = np.zeros((3, 6))
            for i in range(6):
                delta_step = np.zeros(6)
                delta_step[i] = eps
                delta_step_forward = gtsam.Pose3.Expmap(delta_step)
                delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

                poseA_f = pose.compose(delta_step_forward)
                poseA_b = pose.compose(delta_step_backward)

                error_forward = self.get_unary_heading(poseA_f, measurement)
                error_backward = self.get_unary_heading(poseA_b, measurement)
                hdot =  (error_forward - error_backward) / (2*eps)
                H0[:,i] = hdot

            jacobians[0] = H0

        error = self.get_unary_heading(pose, measurement)

        return error

    def error_gps(self, measurement: np.ndarray, this: gtsam.CustomFactor,
              values: gtsam.Values,
              jacobians: Optional[List[np.ndarray]]) -> float:
        """GPS Factor error function
        :param measurement: GPS measurement, to be filled with `partial`
        :param this: gtsam.CustomFactor handle
        :param values: gtsam.Values
        :param jacobians: Optional list of Jacobians
        :return: the unwhitened error
        """

        # Get agent pose
        estimate = values.atPose3(this.keys()[0])

        # Three rows bc [x, y, z] from gps state, six columns bc the state of this node is Pose3
        H = np.zeros((3, 6))
        eps = 1e-6

        # Get Jacobians. Perturb in each direction of the delta vector one at a time.
        for i in range(6):
            delta_step = np.zeros(6)
            delta_step[i] = eps
            delta_step_forward = gtsam.Pose3.Expmap(delta_step)
            delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

            q_exp_forward = estimate.compose(delta_step_forward)
            q_exp_backward = estimate.compose(delta_step_backward)
            h_forward = q_exp_forward.translation()
            h_backward = q_exp_backward.translation()
            error_forward = h_forward - measurement
            error_backward = h_backward - measurement
            hdot =  (error_forward - error_backward) / (2*eps)
            H[:,i] = hdot

        
        error = estimate.translation() - measurement

        if jacobians is not None:
            # Error wrt agent A
            jacobians[0] = H
    

        return error
    

    def error_depth(self, measurement: np.ndarray, this: gtsam.CustomFactor,
             values: gtsam.Values,
             jacobians: Optional[List[np.ndarray]]) -> float:
        """Depth Factor error function
        :param measurement: Landmark measurement, to be filled with `partial`
        :param this: gtsam.CustomFactor handle
        :param values: gtsam.Values
        :param jacobians: Optional list of Jacobians
        :return: the unwhitened error
        """
        pos = values.atPose3(this.keys()[0])

        # Depth error (z position). TODO 1D or 2D array?
        error = pos.translation()[2] - measurement[0]
        # print("depth pred", pos.translation()[2])
        # print("depth meas", pos.translation()[2])

        # print("depth error", error)

        # Jacobians
        if jacobians is not None:
            eps = 1e-6

            # Wrt pose key
            H0 = np.zeros((1, 6))

            # Get Jacobians. Perturb in each direction of the delta vector one at a time.
            for i in range(6):
                delta_step = np.zeros(6)
                delta_step[i] = eps
                delta_step_forward = gtsam.Pose3.Expmap(delta_step)
                delta_step_backward = gtsam.Pose3.Expmap(-delta_step)

                # Perturb pose
                q_exp_forward = pos.compose(delta_step_forward)
                q_exp_backward = pos.compose(delta_step_backward)

                # Extract depth
                h_forward = q_exp_forward.translation()[2]
                h_backward = q_exp_backward.translation()[2]

                # Compute error
                error_forward = h_forward - measurement[0]
                error_backward = h_backward - measurement[0]
                hdot =  (error_forward - error_backward) / (2*eps)
                H0[:,i] = hdot

            jacobians[0] = H0

        return error


    # helper funciton to get H. matrix from rotation and translation
    def HfromRT(self, R, t):
        H = np.eye(4)
        H[:3, :3] = R
        H[:3, 3] = t
        return H




    
    ##################################################################
    ######################## SENSOR CALLBACKS ########################
    ##################################################################


    # orientation from modem
    def imu_callback(self, msg: Imu):
        self.imu_received = True

        self.orientation_covariance = np.array(msg.orientation_covariance).reshape(3, 3)
        # Convert quaternion to rotation matrix
        quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        r = R.from_quat(quat)
        self.orientation_matrix = r.as_matrix()
        #PLOT
        # time = msg.header.stamp.nanosec + msg.header.stamp.sec * 1e9
        # self.plot.add_measurement(DUMMY_IMU_VAL,time,posekey=None,sensor='imu')

        if self.deployed:
            self.q_imu.append(msg)    
        
    # depth sensor
    def depth_callback(self, msg: PoseWithCovarianceStamped):
        # Set the z position and covariance
        self.depth_received = True
        self.position[2] = msg.pose.pose.position.z
        #TODO: find true covariance of depth sensor
        #PLOT
        #time = msg.header.stamp.nanosec + msg.header.stamp.sec * 1e9
        # self.plot.add_measurement(self.position[2],time,posekey=None,sensor='depth')
    
        if self.deployed:
            self.q_depth.append(msg)
    
    # gps (already in x,y from gps_odom.py)
    def gps_callback(self, msg: Odometry):
        self.gps_received = True
        # Get the x, y position and position covariance
        self.position[0] = msg.pose.pose.position.x
        self.position[1] = msg.pose.pose.position.y
        self.gps_z = msg.pose.pose.position.z
        #PLOT
        #time = msg.header.stamp.nanosec + msg.header.stamp.sec * 1e9
        # self.plot.add_measurement(self.position[0],time,sensor='gps')
        # self.x_gps.add_measurement(self.position[0], time)
        # print("gps:", self.position[0], time)
        if not (msg.pose.covariance[0] ==0 or msg.pose.covariance[4]==0 or msg.pose.covariance[8] == 0): #no variance estimate
            self.GPS_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([msg.pose.covariance[0], msg.pose.covariance[4], msg.pose.covariance[8], np.deg2rad(0.5), np.deg2rad(0.5), np.deg2rad(0.5)]))
        if self.deployed :
            self.q_gps.append(msg)
   

    # dvl for new odometry
    def dvl_callback(self, msg: PoseWithCovarianceStamped):

        self.dvl_received = True
        # Get the x, y, z position
        self.dvl_position[0] = msg.pose.pose.position.x
        self.dvl_position[1] = msg.pose.pose.position.y
        self.dvl_position[2] = msg.pose.pose.position.z
        self.dvl_std = msg.pose.covariance[0]
        self.dvl_quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        self.dvl_time = msg.header.stamp.nanosec + msg.header.stamp.sec * 1e9
        self.dvl_stamp = msg.header.stamp
        #6x6 matrix
        for i in range(3):
            for j in range(3):
                self.position_covariance[i,j]=msg.pose.covariance[i+j*6]
        self.DVL_NOISE = gtsam.noiseModel.Diagonal.Sigmas(np.array([self.position_covariance[0,0], self.position_covariance[1,1], self.position_covariance[2,2], np.deg2rad(0.5), np.deg2rad(0.5), np.deg2rad(0.5)]))

    

    ##################################################################
    ######################## SIGNAL TO BEGIN #########################
    ##################################################################

    def init_callback(self, request, response):
        bool_value = request.data
        #TODO: Handle a false value that can reset of the factor graph

        if self.dvl_received and (self.gps_received or not self.get_parameter("using_gps").get_parameter_value().bool_value) and (self.depth_received or not self.get_parameter("using_depth").get_parameter_value().bool_value) and (self.imu_received or not self.get_parameter("using_imu").get_parameter_value().bool_value) and not self.deployed:
            # Store current state as the initial state
            self.get_logger().info("starting graph")
            self.init_state = {
                'position': self.position.copy(),
                'orientation_matrix': self.orientation_matrix,
                'dvl_position': self.dvl_position.copy()
            }

            ############## DO we need to get the orientation matrix of the DVL?  ############

            H = self.HfromRT(self.init_state['orientation_matrix'], self.init_state['position']) #initial orientation matrix

            self.agent = Agent(H)

            # self.dvl_pose_current = gtsam.Pose3(H)
            
            self.poseKey = int(1)
            self.prevPoseKey = self.poseKey

            
            #TODO: change noise model?
            priorFactor = gtsam.PriorFactorPose3(self.agent.poseKey, self.agent.pose_world, self.DVL_NOISE)
            print("x prior: ", self.agent.pose_world.translation()[0])
            print("y prior: ", self.agent.pose_world.translation()[1])

            print("x gps: ", self.init_state['position'][0])
            print("y gps: ", self.init_state['position'][1])
            print("x dvl: ", self.init_state['dvl_position'][0])
            print("y dvl: ", self.init_state['dvl_position'][1])


            self.graph.push_back(priorFactor)
            self.initialEstimate.insert(self.agent.poseKey, self.agent.pose_world)


            self.poseKey_to_time[self.agent.poseKey] = self.dvl_time
            self.gps_last_pose_key = self.agent.poseKey
            self.depth_last_pose_key = self.agent.poseKey
            self.imu_last_pose_key = self.agent.poseKey


            # self.dvl_position_last = self.agent.pose_world


            dvl_position = self.init_state['dvl_position']
            r = R.from_quat(self.dvl_quat)
            self.dvl_orientation_matrix = r.as_matrix()
            self.dvl_position_last = gtsam.Pose3(self.HfromRT(self.dvl_orientation_matrix , dvl_position))
        
            self.get_logger().info("Factor Graph: Initial state has been set.")

            # Plot
            # self.x_dvl.add_measurement(self.position[0], self.dvl_time)
            print('Initial State DVL:', self.position[0])


            self.update()
            self.publish_vehicle_status()

            self.deployed = True


            # TODO: Create a warning if DVL values are higher than a threshold 
            # We would know that the DVL was not reset recently
            threshold = 0.1
            if any(value > threshold for value in dvl_position):
                self.get_logger().info("Initial DVL Position is higher than threshold. Was DR reset?")

            t = TransformStamped()
            # Read message content and assign it to
            # corresponding tf variables
            t.header.stamp = self.dvl_stamp
            print("stamp added", self.dvl_stamp.sec, self.dvl_stamp.nanosec)
            t.header.frame_id = 'world'
            t.child_frame_id = 'odom'
            # DVL needs to be reset DR shortly before this happens 
            t.transform.translation.x = self.init_state['position'][0] 
            t.transform.translation.y = self.init_state['position'][1] 
            t.transform.translation.z = self.init_state['position'][2] 

            # We need to know the global orientation when this was started so DVL DR should have been reset recently
            q = R.from_matrix(self.init_state['orientation_matrix']).as_quat()
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            self.tf_broadcaster.sendTransform(t)

            response.success = True
            response.message = "Factor Graph Started"
        elif self.deployed:
            self.get_logger().info("already deployed")
            response.success = True
            response.message = f"IMU: {self.imu_received}, GPS: {self.gps_received}, DVL:{self.dvl_received}, Depth: {self.depth_received}"
        else:
            dummy = 1
            self.get_logger().info("Have not received all necessary sensor inputs to begin")
            self.get_logger().info(f"IMU: {self.imu_received}, GPS: {self.gps_received}, DVL:{self.dvl_received}, Depth: {self.depth_received}")
            response.success = False
            response.message = f"IMU: {self.imu_received}, GPS: {self.gps_received}, DVL:{self.dvl_received}, Depth: {self.depth_received}"
        
        return response


    
    ##################################################################
    ########### algorithms for time syncing measurements #############
    ##################################################################

    def unary_assignment(self, sensor): #time-syncs measurements before inserting them into the factor graph

        if sensor == 'gps':
     
            in_future = False
            new_id = int(self.agent.poseKey)    #The posekey id that you will start searching at
            curr_time = self.poseKey_to_time[new_id] 
            while(len(self.q_gps) > 1 and in_future is False and new_id > 1):       #If measurement in queue and the oldest measurment is later than current posekey
                #print("new_id:%d"%new_id)
                oldest_measurement_time = (self.q_gps[0].header.stamp.sec * 1_000_000_000 + self.q_gps[0].header.stamp.nanosec) 
                #print("oldest measurment time: %d\n"%oldest_measurement_time)
                next_measurement_time = (self.q_gps[1].header.stamp.sec * 1_000_000_000 + self.q_gps[1].header.stamp.nanosec ) 
                #print("next measurment time: %d\n"%next_measurement_time)
                # print('q > 1')
                #print('gps queue', len(self.q_gps))
                if(oldest_measurement_time < curr_time):
                    #print('oldest measurement time < curr time')
                    
                    newer_key_time = self.poseKey_to_time[int(new_id)] 
                    # print("newer key time: %d\n"%newer_key_time)
                    older_key_time = self.poseKey_to_time[int(new_id - 1)]
                    # print("older key time: %d\n"% older_key_time)
                    time_to_current = abs(newer_key_time - oldest_measurement_time) 
                    # print("time to current: %d\n"%time_to_current)
                    time_to_previous = abs(older_key_time - oldest_measurement_time) 
                    # print("time to previous: %d\n"%time_to_previous)

                    if(time_to_current > time_to_previous):
                        #print('time to current is longer than time to prev')
                        new_id -= 1
                        if(new_id == self.gps_last_pose_key):
                            # print('pop, last gps key added:', self.gps_last_pose_key)
                            self.q_gps.pop(0)
                            new_id = int(self.agent.poseKey)
                    else:
                            
                        #print('time to current is shortest')

                        time_old_to_pose = abs(oldest_measurement_time - newer_key_time)
                        # print("time old to pose: %d\n"%time_old_to_pose)
                        time_next_to_pose = abs(next_measurement_time - newer_key_time)
                        # print("time next to pose: %d\n"%time_old_to_pose)

                        if(next_measurement_time < newer_key_time):
                        # Take care of where next measurement is not past next node
                            self.q_gps.pop(0)
                            #print('popped because newer measurement is not past next node')
                        
                        elif(time_old_to_pose > time_next_to_pose):
                        # Take care of case where next measurement is better
                            self.q_gps.pop(0)
                            #print('popped because newer measurement is closer')
                            new_id = int(self.agent.poseKey)

                        else:
                            if(new_id == self.gps_last_pose_key):
                                #print('popped bad this should not happen')
                                self.q_gps.pop(0)
                                new_id = int(self.agent.poseKey)
                            else:
                                #Actually add the  factor
                                msg = self.q_gps.pop(0)

                                
                                gps_meas = gtsam.Point3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z)
                                if(self.position[2] > DEPTH_THRESHOLD):
                                    self.graph.add(gtsam.CustomFactor(self.GPS_NOISE, [new_id], partial(self.error_gps, gps_meas)))
                                # self.get_logger().info("added gps unary %d"%new_id)
                                
                                #PLOT
                                # time = msg.header.stamp.nanosec + msg.header.stamp.sec * 1e9
                                # self.plot.add_measurement(msg.pose.pose.position.x,time, posekey=new_id, sensor='gps' )
                                # self.x_gps.add_measurement(msg.pose.pose.position.x,time, pose_key=new_id)
                        
                                self.gps_last_pose_key = new_id
                                #print('last gps key is now: ', self.gps_last_pose_key)
                                new_id = int(self.agent.poseKey)

                else:
                    in_future = True



        elif sensor == 'depth' or sensor == 'imu':
            
            msg_queue = []
            if sensor == 'depth':
                # self.get_logger().info("depth unary")
                msg_queue = self.q_depth # not a copy, the reference
                last_pose_key = self.depth_last_pose_key
            elif sensor == 'imu':
                # self.get_logger().info("imu unary")
                msg_queue = self.q_imu # not a copy, the reference
                last_pose_key = self.imu_last_pose_key



            in_future = False
            new_id = int(self.agent.poseKey)    #The posekey id that you will start searching at
            curr_time = self.poseKey_to_time[new_id] 
            while(len(msg_queue) > 1 and in_future is False):       #If measurement in queue and the oldest measurment is later than current posekey
                # print("new_id:%d"%new_id)
                oldest_measurement_time = (msg_queue[0].header.stamp.sec * 1_000_000_000 + msg_queue[0].header.stamp.nanosec) 
                # print("oldest measurment time: %d\n"%oldest_measurement_time)
                next_measurement_time = (msg_queue[1].header.stamp.sec * 1_000_000_000 + msg_queue[1].header.stamp.nanosec) 
                # print("next measurment time: %d\n"%next_measurement_time)
                # print('q > 1')
                # print(len(msg_queue))
                if(oldest_measurement_time < curr_time):
                    # print('oldest measurement time < curr time')
                    
                    newer_key_time = self.poseKey_to_time[int(new_id)] 
                    # print("newer key time: %d\n"%newer_key_time)
                    older_key_time = self.poseKey_to_time[int(new_id - 1)]
                    # print("older key time: %d\n"% older_key_time)
                    time_to_current = abs(newer_key_time - oldest_measurement_time) 
                    # print("time to current: %d\n"%time_to_current)
                    time_to_previous = abs(older_key_time - oldest_measurement_time) 
                    # print("time to previous: %d\n"%time_to_previous)

                    if(time_to_current > time_to_previous):
                        # print('time to current is longer than time to prev')
                        new_id -= 1
                        if(new_id == last_pose_key):
                            # print('pop')
                            msg_queue.pop(0)
                            new_id = int(self.agent.poseKey)
                    else:
                        # print('time to current is shortest')

                        time_old_to_pose = abs(oldest_measurement_time - newer_key_time)
                        # print("time old to pose: %d\n"%time_old_to_pose)
                        time_next_to_pose = abs(next_measurement_time - newer_key_time)
                        # print("time next to pose: %d\n"%time_old_to_pose)

                        if(next_measurement_time < newer_key_time):
                        # Take care of where next measurement is not past next node
                            msg_queue.pop(0)
                        
                        elif(time_old_to_pose > time_next_to_pose):
                        # Take care of case where next measurement is better
                            msg_queue.pop(0)
                            new_id = self.agent.poseKey

                        else:
                            #Actually add the  factor
                            msg = msg_queue.pop(0)


                            if sensor == 'depth':
                                self.graph.add(gtsam.CustomFactor(self.DEPTH_NOISE, [new_id], partial(self.error_depth, [np.array([msg.pose.pose.position.z])])))
                                # time = msg.header.stamp.nanosec + msg.header.stamp.sec * 1e9
                                # self.plot.add_measurement(DUMMY_DEPTH_VAL,time, posekey=new_id, sensor='depth' )
                                # self.get_logger().info("added depth unary %d"% new_id)
                
                                
                            elif sensor == 'imu': #magnetometer reading
                                quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
                                r = R.from_quat(quat)
                                orientation_matrix = r.as_matrix()
                                # Get the orientation covariance
                                orientation_meas = gtsam.Pose3(self.HfromRT(orientation_matrix, [0,0,0])).rotation()
                                self.graph.add(gtsam.CustomFactor(self.UNARY_HEADING_NOISE, [new_id], partial(self.error_unary_heading, [orientation_meas])))
                                #time = msg.header.stamp.nanosec + msg.header.stamp.sec * 1e9
                                # self.plot.add_measurement(DUMMY_IMU_VAL,time, posekey=new_id, sensor='imu' )
                                # self.get_logger().info("added imu unary %d" % new_id)

                            last_pose_key = new_id
                            new_id = self.agent.poseKey
                        
                else:
                    in_future = True


                if sensor == 'imu':
                    self.imu_last_pose_key = last_pose_key
                elif sensor == 'depth':
                    self.depth_last_pose_key = last_pose_key

            if sensor == 'depth':
                self.depth_last_pose_key = int(last_pose_key)
            elif sensor == 'imu':
                self.imu_last_pose_key = int(last_pose_key)



    ##################################################################
    ####################### factor graph stuff #######################
    ##################################################################

        # update at every tick of the timer (after between factor and checking for unary factors)
    def update(self):
        self.factorStatus=self.isam.update(self.graph, self.initialEstimate) #initial estimate becomes the dvl transformed by 
        self.result = self.isam.calculateEstimate()
        self.xyz = self.result.atPose3(self.agent.poseKey).translation()
        self.initialEstimate.clear()
        maxFactorNum=self.get_parameter("max_factor_graph_size").get_parameter_value().integer_value
        if(self.graph.size()>maxFactorNum):
            self.graph.resize(maxFactorNum) #cuts size to max size
        #print("poseKey number: ", self.agent.poseKey)
        #print("translation ", self.result.atPose3(self.agent.poseKey).translation())
        #print("latest pose x result: ", self.result.atPose3(self.agent.poseKey).translation()[0])
        #print("latest pose y result: ", self.result.atPose3(self.agent.poseKey).translation()[1])
        #print("1st pose x result: ", self.result.atPose3(int(1)).translation()[0])
        #print("1st pose y result: ", self.result.atPose3(int(1)).translation()[1])

    def factor_graph_timer(self):
        # Timer callback function
        # self.get_logger().info('factor_graph_timer function is called')
        if self.deployed:
            dvl_position = self.dvl_position
            r = R.from_quat(self.dvl_quat)
            self.dvl_orientation_matrix = r.as_matrix()
            self.dvl_pose_current = gtsam.Pose3(self.HfromRT(self.dvl_orientation_matrix , dvl_position))
            
            # get the pose2 wrt (with respect to) pose1
            H_pose2_wrt_pose1= self.dvl_position_last.inverse().compose(self.dvl_pose_current)

            # add the odometry
            self.agent.prevPoseKey = int(self.agent.poseKey)
            self.agent.poseKey = int(1 + self.agent.poseKey)
            #print('posekey:', self.agent.poseKey)

            #transforms previous result by the dvl's origin
            global_pose_current_init = self.result.atPose3(self.agent.prevPoseKey).compose(H_pose2_wrt_pose1)

            # initial estimate = previous estimate transformed by dvl origin
            self.initialEstimate.insert(self.agent.poseKey, global_pose_current_init)
            self.graph.add(gtsam.BetweenFactorPose3(self.agent.prevPoseKey, self.agent.poseKey, H_pose2_wrt_pose1, self.DVL_NOISE)) #binary factor for dvl odom
            self.poseKey_to_time[int(self.agent.poseKey)] = self.dvl_time

            #PLOT 
            #delta_local = H_pose2_wrt_pose1.translation()
            #delta_global = np.matmul(self.init_state['orientation_matrix'], delta_local)
            #delta_x_local = delta_local[0]
            #delta_x_global = delta_global[0]
            # self.plot.add_delta_measurement(delta_x, self.dvl_time,self.agent.poseKey)
            # self.x_dvl.add_delta_measurement(delta_x_global, self.dvl_time)
            # self.plot.update_plot()
            #print('COMPARISON:',(dvl_position[0]- self.prev_x), delta_x_local, 'global:', delta_x_global)
            
            self.prev_x = dvl_position[0]
            # self.prev_x = delta_x
            #Time sync measurements
            # IMU unary factor
            if self.get_parameter("using_imu").get_parameter_value().bool_value:
                self.unary_assignment('imu')
            # print('out of imu')
            # Depth unary factor
            if self.get_parameter("using_depth").get_parameter_value().bool_value:
                self.unary_assignment('depth')
            # print('out of depth')
            # GPS unary factor
            if self.get_parameter("using_gps").get_parameter_value().bool_value:
                self.unary_assignment('gps')
            # print('out of gps')
            self.dvl_position_last = self.dvl_pose_current

            self.update()
            self.publish_vehicle_status()

            #PLOT
            # self.x_output.add_measurement(self.xyz[0], self.dvl_time,self.agent.poseKey)
            # for i in range (len(self.x_output.pose_keys)):
            #     self.x_output.values[i] = self.result.atPose3(self.x_output.pose_keys[i]).translation()[0]
            # self.x_plot.update_plot()

    ##############################
    #### DATA PUBLISHING FUNC ####
    ##############################

    def publish_vehicle_status(self):
        
        # # Set the orientation in the message
        r = R.from_matrix(self.orientation_matrix)
        quat = r.as_quat()
        self.odom_msg.pose.pose.orientation.x = quat[0]
        self.odom_msg.pose.pose.orientation.y = quat[1]
        self.odom_msg.pose.pose.orientation.z = quat[2]
        self.odom_msg.pose.pose.orientation.w = quat[3]

        # # Set the position in the message
        
        # this is the only gtsam output right now, orientation and depth are raw from the sensors
        self.odom_msg.pose.pose.position.x = self.xyz[0]
        self.odom_msg.pose.pose.position.y = self.xyz[1]
        #print("vehicle status xyz[0]: ", self.xyz[0])

        #print("vehicle status x: ", self.odom_msg.pose.pose.position.x)


        self.odom_msg.pose.pose.position.z = self.position[2]

        # Set the covariance
        #0.2 is z covariance TODO: measure covariance of depth sensor (?)
        self.odom_msg.pose.covariance = [
            self.position_covariance[0, 0], 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, self.position_covariance[1, 1], 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.2, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, self.orientation_covariance[0, 0], 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, self.orientation_covariance[1, 1], 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, self.orientation_covariance[2, 2]
        ]


        # Publish the vehicle status
        self.vehicle_status_pub.publish(self.odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FactorGraphNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
