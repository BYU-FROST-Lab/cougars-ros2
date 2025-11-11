#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import os

class ParameterReloader(Node):
    def __init__(self):
        super().__init__('parameter_reloader')
        
        # Declare parameters
        self.declare_parameter('param_file_path', '/home/frostlab/base_station/mission_control/params/coug2_params.yaml')
        self.declare_parameter('target_nodes', ['/coug2/coug_controls', '/coug2/coug_kinematics', '/coug2/cougars_coms', '/coug2/manual_mission'])
        
        # Get parameters
        self.param_file_path = self.get_parameter('param_file_path').get_parameter_value().string_value
        self.target_nodes = self.get_parameter('target_nodes').get_parameter_value().string_array_value
        
        # Create subscriber for reload trigger
        self.reload_sub = self.create_subscription(
            Empty,
            'reload_parameters',
            self.reload_callback,
            10
        )
        
        self.get_logger().info(f"Parameter Reloader Node started")
        self.get_logger().info(f"Param file: {self.param_file_path}")
        self.get_logger().info(f"Target nodes: {self.target_nodes}")
        self.get_logger().info("Listening for reload commands on: /reload_parameters")
        
    def reload_callback(self, msg):
        """Handle reload request"""
        self.get_logger().info("🔄 Received reload request")
        self.reload_parameters()
    
    def reload_parameters(self):
        """Reload parameters for all target nodes"""
        if not os.path.exists(self.param_file_path):
            self.get_logger().error(f"❌ Parameter file not found: {self.param_file_path}")
            return
        
        success_count = 0
        total_count = len(self.target_nodes)
        
        for node_name in self.target_nodes:
            if self.reload_node_params(node_name):
                success_count += 1
        
        self.get_logger().info(f"✅ Parameter reload complete! {success_count}/{total_count} nodes updated")
    
    def reload_node_params(self, node_name):
        """Reload parameters for a single node using ros2 param load"""
        try:
            # Check if node is running using ros2 node list
            check_cmd = f"ros2 node list | grep -q '^{node_name}$'"
            check_result = os.system(check_cmd)
            
            if check_result != 0:
                self.get_logger().warn(f"⚠️  Node not running: {node_name}")
                return False
            
            # Use ros2 param load to reload parameters
            cmd = f"timeout 10s ros2 param load {node_name} {self.param_file_path} 2>/dev/null"
            result = os.system(cmd)
            
            if result == 0:
                self.get_logger().info(f"✅ Updated: {node_name}")
                return True
            else:
                self.get_logger().warn(f"⚠️  Failed to update: {node_name}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"❌ Error updating {node_name}: {e}")
            return False

def main(args=None):
    rclpy.init(args=args)
    reloader = ParameterReloader()
    
    try:
        rclpy.spin(reloader)
    except KeyboardInterrupt:
        reloader.get_logger().info("Parameter reloader node interrupted")
    finally:
        reloader.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
