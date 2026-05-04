
#include "rclcpp/rclcpp.hpp"
#include "geographic_msgs/msg/route_network.hpp"
#include "geographic_msgs/msg/way_point.hpp"
#include "geographic_msgs/msg/key_value.hpp"
#include "cougars_interfaces/msg/mission_feedback.hpp"
#include "cougars_interfaces/msg/waypoint_feedback.hpp"
#include "cougars_interfaces/msg/system_control.hpp"

/*
This node takes a mission, or a list of waypoints, and iteratively publishes them to the waypoint manager.
It also listens for feedback on the current waypoint and sends mission feedback accordingly.
The node starts publishing waypoints when it receives a startup message with start.data == true.
It tracks which waypoints have been completed or skipped and includes that information in the mission feedback messages.

Publishers:
- "waypoint" (geographic_msgs::msg::WayPoint): Publishes the current waypoint to the waypoint manager.
- "mission_feedback" (cougars_interfaces::msg::MissionFeedback): Publishes feedback on the mission progress, including
                    which waypoints have been completed, skipped, and are remaining.

Subscribers:
- "mission" (geographic_msgs::msg::RouteNetwork): Subscribes to the mission, which contains a list of waypoints and their properties.
- "waypoint_feedback" (cougars_interfaces::msg::WaypointFeedback): Subscribes to feedback on the current waypoint, which
                    indicates whether the waypoint was arrived at, skipped, or if there was an error.
- "startup" (cougars_interfaces::msg::SystemControl): Subscribes to startup messages to know when to start the mission. 
                    When a startup message with start.data == true is received, the node begins
                    
Variables:
- waypoint_list (std::vector<geographic_msgs::msg::WayPoint>): Stores the list of waypoints from the mission.
- current_waypoint_index (size_t): Tracks the index of the current waypoint being processed.
- mission_state (uint8_t): Tracks the current state of the mission (idle, running, complete, aborted).
- start_time (rclcpp::Time): Records the time when the mission was started to calculate elapsed time for feedback.
- skipped_waypoint_indices (std::set<size_t>): Tracks which waypoints were skipped to include that information in the mission feedback.
*/

class WaypointIterator : public rclcpp::Node
{
public:
    WaypointIterator() : Node("waypoint_follower") {

        // subscribe to mission with message type route network
        this->mission_sub_ = this->create_subscription<geographic_msgs::msg::RouteNetwork>(
            "mission", 10, std::bind(&WaypointIterator::missionCallback, this, std::placeholders::_1));

        // subscription to mission feedback, used to send updates on mission progress
        this->waypoint_feedback_sub_ = this->create_subscription<cougars_interfaces::msg::WaypointFeedback>(
            "waypoint_feedback", 10, std::bind(&WaypointIterator::waypointFeedbackCallback, this, std::placeholders::_1));

        // subscription to startup
        this->startup_sub_ = this->create_subscription<cougars_interfaces::msg::SystemControl>(
            "startup", 10, std::bind(&WaypointIterator::startupCallback, this, std::placeholders::_1));

        // publishes waypoints to the waypoint manager
        this->waypoint_pub_ = this->create_publisher<geographic_msgs::msg::WayPoint>("waypoint", 10);

        // publishes mission feedback
        this->mission_feedback_pub_ = this->create_publisher<cougars_interfaces::msg::MissionFeedback>("mission_feedback", 10);

        this->mission_state = cougars_interfaces::msg::MissionFeedback::STATE_IDLE;

        this->start_time = rclcpp::Time(0);

    }

    void missionCallback(const geographic_msgs::msg::RouteNetwork::SharedPtr msg) {
        // Handle mission message
        std::unordered_map<std::string, std::string> route_props_map = getKeyValue(msg->props);
        waypoint_list.clear();

        // Populates any missing properties in the waypoints with the mission-level properties, so that the waypoints have all necessary information for the waypoint manager
        for (geographic_msgs::msg::WayPoint& wp : msg->points) {
            std::unordered_map<std::string, std::string> props_map = getKeyValue(wp.props);
            if (props_map.find("speed") == props_map.end()){
                geographic_msgs::msg::KeyValue speed_kv;
                speed_kv.key = "speed";
                speed_kv.value = route_props_map["speed"];
                wp.props.push_back(speed_kv);
            }
            if (props_map.find("slip") == props_map.end()){
                geographic_msgs::msg::KeyValue slip_kv;
                slip_kv.key = "slip";
                slip_kv.value = route_props_map["slip"];
                wp.props.push_back(slip_kv);
            }
            if (props_map.find("cap") == props_map.end()){
                geographic_msgs::msg::KeyValue cap_kv;
                cap_kv.key = "cap";
                cap_kv.value = route_props_map["cap"];
                wp.props.push_back(cap_kv);
            }
            
           
            waypoint_list.push_back(wp);
        }
    }

    void waypointFeedbackCallback(const cougars_interfaces::msg::WaypointFeedback::SharedPtr msg) {
        // Handle waypoint feedback message
        if (this->mission_state != cougars_interfaces::msg::MissionFeedback::STATE_RUNNING) {
            RCLCPP_WARN(this->get_logger(), "Received waypoint feedback while mission is not active. Ignoring.");
            return;
        }
        this->current_wp_feedback = *msg;
        // If state is arrived or skipped, move on to the next waypoint and pulish it
        if (msg->state == cougars_interfaces::msg::WaypointFeedback::STATE_ARRIVED || msg->state == cougars_interfaces::msg::WaypointFeedback::STATE_SKIPPED) {
            // If waypoint got skipped, store in skipped_waypoint_indeces  
            if (msg->state == cougars_interfaces::msg::WaypointFeedback::STATE_SKIPPED) {
                skipped_waypoint_indices.insert(this->current_waypoint_index);
            }
            // Move to the next waypoint
            this->current_waypoint_index++;
            // If complete change mission state and publish feedback, otherwise publish the next waypoint
            if (this->current_waypoint_index >= this->waypoint_list.size()) {
                RCLCPP_INFO(this->get_logger(), "Mission completed! All waypoints have been reached or skipped.");
                this->mission_state = cougars_interfaces::msg::MissionFeedback::STATE_COMPLETE;
                publishMissionFeedback();
                return;
            }
            this->publishCurrentWaypoint();
            RCLCPP_INFO(this->get_logger(), "Waypoint %ld completed. Moving to waypoint %ld.", this->current_waypoint_index - 1, this->current_waypoint_index); 
        }
        publishMissionFeedback();
    }

    void startupCallback(const cougars_interfaces::msg::SystemControl::SharedPtr msg) {
        // Handle startup message
        if (this->mission_state != cougars_interfaces::msg::MissionFeedback::STATE_RUNNING && msg->start.data == true) {
            this->start_time = this->now();
            this->mission_state = cougars_interfaces::msg::MissionFeedback::STATE_RUNNING;
            this->current_waypoint_index = 0;
            this->publishCurrentWaypoint();
            RCLCPP_INFO(this->get_logger(), "Mission started. First waypoint published.");
        } else {
            if (msg->start.data == true) {
                RCLCPP_WARN(this->get_logger(), "Mission is already active. Ignoring startup command.");
            } else {
                RCLCPP_DEBUG(this->get_logger(), "Mission is being stopped.");
                this->mission_state = cougars_interfaces::msg::MissionFeedback::STATE_ABORTED;
            }
        }
    }

    void publishMissionFeedback() {
        // creates mission feedack message
        cougars_interfaces::msg::MissionFeedback feedback_msg;
        feedback_msg.state = this->mission_state;
        feedback_msg.waypoints_completed = this->current_waypoint_index;
        feedback_msg.waypoints_total = this->waypoint_list.size();
        feedback_msg.elapsed_time = (this->now() - this->start_time).seconds();
        feedback_msg.current = this->current_wp_feedback;
        // Finds the ID's for each waypoint and adds them to the appropriate list completed, skipped, or are remaining
        for (size_t i = 0; i < this->current_waypoint_index; i++) {
            if (skipped_waypoint_indices.find(i) != skipped_waypoint_indices.end()) {
                feedback_msg.skipped.push_back(waypoint_list[i].id);
            } else {
                feedback_msg.completed.push_back(waypoint_list[i].id);
            }
        }
        for (size_t i = this->current_waypoint_index; i < this->waypoint_list.size(); i++) {
            feedback_msg.remaining.push_back(waypoint_list[i].id);
        }

        this->mission_feedback_pub_->publish(feedback_msg);
    }

    void publishCurrentWaypoint() {
        if (current_waypoint_index < waypoint_list.size()) {
            waypoint_pub_->publish(waypoint_list[current_waypoint_index]);
        } else {
            RCLCPP_INFO(this->get_logger(), "All waypoints have been published.");
        }
    }

    // used for parsing the properties in the mission and waypoints, returned un an unordered map for easy access
    template<typename Container>
    std::unordered_map<std::string, std::string> getKeyValue(const Container &props) {
            std::unordered_map<std::string, std::string> key_value_map;
            for (const auto& prop : props) {
                key_value_map[prop.key] = prop.value;
            }
            return key_value_map;
    }

private:

    rclcpp::Subscription<geographic_msgs::msg::RouteNetwork>::SharedPtr mission_sub_;
    rclcpp::Subscription<cougars_interfaces::msg::SystemControl>::SharedPtr startup_sub_;
    rclcpp::Subscription<cougars_interfaces::msg::WaypointFeedback>::SharedPtr waypoint_feedback_sub_;

    rclcpp::Publisher<geographic_msgs::msg::WayPoint>::SharedPtr waypoint_pub_;
    rclcpp::Publisher<cougars_interfaces::msg::MissionFeedback>::SharedPtr mission_feedback_pub_;

    std::vector<geographic_msgs::msg::WayPoint> waypoint_list;
    cougars_interfaces::msg::WaypointFeedback current_wp_feedback = cougars_interfaces::msg::WaypointFeedback(); // Store the most recent waypoint feedback message
    size_t current_waypoint_index = 0;

    uint8_t mission_state;

    // start time
    rclcpp::Time start_time;

    std::set<size_t> skipped_waypoint_indices;  // Track which waypoints were skipped

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WaypointIterator>()); 
    rclcpp::shutdown();
    return 0;
}