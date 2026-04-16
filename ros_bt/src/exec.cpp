/*
   This is a demo for BehaviorTree.CPP BehaviorTree.ROS2 
   first you must ros2 run the fibonacci and add_two_ints servers 
   the run demo and it will go through the behaviour tree
 */
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/bt_service_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "nav2_msgs/action/wait.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "dynamixel_sdk_custom_interfaces/msg/set_position.hpp"


namespace chr = std::chrono;
using namespace BT;

using PoseMsg = geometry_msgs::msg::Pose;
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

// SyncActionNode (synchronous action) with an input port.
class SaySomething : public SyncActionNode
{
    public:
        // If your Node has ports, you must use this constructor signature 
        SaySomething(const std::string& name, const NodeConfig& config)
            : SyncActionNode(name, config)
        { }

        // It is mandatory to define this STATIC method.
        static PortsList providedPorts()
        {
            // This action has a single input port called "message"
            return { InputPort<std::string>("message") };
        }

        // Override the virtual function tick()
        NodeStatus tick() override
        {
            Expected<std::string> msg = getInput<std::string>("message");
            // Check if expected is valid. If not, throw its error
            if (!msg)
            {
                throw RuntimeError("missing required input [message]: ", 
                        msg.error() );
            }
            // use the method value() to extract the valid message.
            std::cout << "Robot says: " << msg.value() << std::endl;
            return NodeStatus::SUCCESS;
        }
};
class MapFinished : public SyncActionNode
{
    public:
        // If your Node has ports, you must use this constructor signature 
        MapFinished(const std::string& name, const NodeConfig& config)
            : SyncActionNode(name, config)
        { }

        // It is mandatory to define this STATIC method.
        static PortsList providedPorts()
        {
            // This action has a single input port called "message"
            return { InputPort<OccupancyGrid>("map") };
        }

        // Override the virtual function tick()
        NodeStatus tick() override
        {
            Expected<OccupancyGrid> msg = getInput<OccupancyGrid>("map");
            // Check if expected is valid. If not, throw its error
            if (!msg)
            {
                throw RuntimeError("missing required input [map]: ", 
                        msg.error() );
            }
            auto map=msg.value();
            unsigned n=map.info.width*map.info.height;
            unsigned cnt=0;
            std::cout<<"Map info: "<< map.info.origin.position.x << ", " 
                << map.info.origin.position.x << ","<<n<<std::endl;
            for(unsigned i=0;i<n;++i) {
                if(map.data[i] == -1) {
                    cnt++;
                }
            }
            float percent_done = (float)cnt/(float)n;
            std::cout << "Map is "<< percent_done << " known"<<std::endl;
            if(percent_done > 0.9) {
                std::cout << "Map is finished!\n";
                return NodeStatus::SUCCESS;
            } else {
                return NodeStatus::FAILURE;
            }
        }
};
class WaitForSeconds : public StatefulActionNode
{
    public:
        WaitForSeconds(const std::string& name, const NodeConfiguration& config)
            : StatefulActionNode(name, config)
        {}
        static PortsList providedPorts()
        {
            return{ InputPort<unsigned>("seconds") };
        }
        NodeStatus onStart() override;
        NodeStatus onRunning() override;
        void onHalted() override;
    private:
        unsigned _seconds;
        chr::system_clock::time_point _completion_time;
};
NodeStatus WaitForSeconds::onStart()
{
    if ( !getInput<unsigned>("seconds", _seconds))
    {
        throw RuntimeError("missing required input [seconds]");
    }
    printf("[ WaitForSeconds: ] seconds = %d\n",_seconds);
    _completion_time = chr::system_clock::now() + chr::milliseconds(_seconds*1000);
    return NodeStatus::RUNNING;
}

NodeStatus WaitForSeconds::onRunning()
{
    std::this_thread::sleep_for(chr::milliseconds(1000));
    if(chr::system_clock::now() >= _completion_time)
    {
        std::cout << "[ WaitForSeconds: FINISHED ]" << std::endl;
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::RUNNING;
}
void WaitForSeconds::onHalted()
{
    printf("[ WaitForSeconds: ABORTED ]");
}

using Spin = nav2_msgs::action::Spin;
class SpinAction: public RosActionNode<Spin>
{
    public:
        SpinAction(const std::string& name,
                   const NodeConfig& conf,
                   const RosNodeParams& params)
                   : RosActionNode<Spin>(name, conf, params)
        {}
        static PortsList providedPorts()
        {
            return providedBasicPorts({InputPort<float>("target_yaw")});
        }
        bool setGoal(RosActionNode::Goal& goal) override 
        {
            getInput("target_yaw", goal.target_yaw);
            return true;
        }
        NodeStatus onResultReceived(const WrappedResult& wr) override
        {
            // FIX: Used node_.lock()
            RCLCPP_INFO(node_.lock()->get_logger(), "Spin action Complete");
            return NodeStatus::SUCCESS;
        }
        virtual NodeStatus onFailure(ActionNodeErrorCode error) override
        {
            // FIX: Used node_.lock()
            RCLCPP_ERROR(node_.lock()->get_logger(), "Error: %d", error);
            return NodeStatus::FAILURE;
        }
        NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
        {
            return NodeStatus::RUNNING;
        }
};

using NavigateToPose = nav2_msgs::action::NavigateToPose;
class NavigateToPoseAction: public RosActionNode<NavigateToPose>
{
    public:
        NavigateToPoseAction(const std::string& name,
                const NodeConfig& conf,
                const RosNodeParams& params)
                : RosActionNode<NavigateToPose>(name, conf, params)
        {}
        static PortsList providedPorts()
        {
            return providedBasicPorts({
                    InputPort<PoseMsg>("pose")
                    });
        }
        bool setGoal(RosActionNode::Goal& goal) override 
        {
            getInput("pose", goal.pose.pose);
            printf("Pose %f,%f\n",goal.pose.pose.position.x, goal.pose.pose.position.y);
            goal.pose.header.frame_id="map";
            return true;
        }
        NodeStatus onResultReceived(const WrappedResult& wr) override
        {
            // FIX: Used node_.lock()
            RCLCPP_INFO(node_.lock()->get_logger(), "NavigateToPose Goal reached!");
            return NodeStatus::SUCCESS;
        }
        virtual NodeStatus onFailure(ActionNodeErrorCode error) override
        {
            // FIX: Used node_.lock()
            RCLCPP_ERROR(node_.lock()->get_logger(), "Error in Result Recieved: %d", error);
            return NodeStatus::SUCCESS;
        }
        NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
        {
            return NodeStatus::RUNNING;
        }
};

using GetMap = nav_msgs::srv::GetMap;
class MapService: public RosServiceNode<GetMap>
{
    public:
        MapService(const std::string& name,
                const NodeConfig& conf,
                const RosNodeParams& params)
            : RosServiceNode<GetMap>(name, conf, params)
        {}
        static PortsList providedPorts()
        {
            return providedBasicPorts({OutputPort<OccupancyGrid>("map")});
        }
        bool setRequest(Request::SharedPtr& request) override
        {
            return true;
        }
        NodeStatus onResponseReceived(const Response::SharedPtr& response) override
        {
            // FIX: Used node_.lock()
            RCLCPP_INFO(node_.lock()->get_logger(), "Got a map!!!!!");
            setOutput("map",response->map);
            return NodeStatus::SUCCESS;
        }
        virtual NodeStatus onFailure(ServiceNodeErrorCode error) override
        {
            // FIX: Used node_.lock()
            RCLCPP_ERROR(node_.lock()->get_logger(), "Error: %d", error);
            return NodeStatus::FAILURE;
        }
};

class FindFrontier : public SyncActionNode
{
    private:
        PoseMsg _current_active_goal;
        bool _has_active_goal = false;
        const double COMMITMENT_THRESHOLD = 2.0;

    public:
        // If your Node has ports, you must use this constructor signature 
        FindFrontier(const std::string& name, const NodeConfig& config)
            : SyncActionNode(name, config)
        { }

        // It is mandatory to define this STATIC method.
        static PortsList providedPorts()
        {
            return {OutputPort<PoseMsg>("goal_pose"), 
                    InputPort<PoseMsg>("current_pose"),
                    InputPort<OccupancyGrid>("map") };
        }
        PoseMsg _current_pose;

        PoseMsg _find_frontier(OccupancyGrid map, PoseMsg pose) {
            auto resolution = map.info.resolution; 
            auto og_x = map.info.origin.position.x;
            auto og_y = map.info.origin.position.y;
            
            // Convert robot pose to pixel coordinates
            unsigned pose_x = (pose.position.x - og_x) / resolution;
            unsigned pose_y = (pose.position.y - og_y) / resolution;
            
            int max_x = map.info.width;
            int max_y = map.info.height;
            
            // Initialize best goal to current position (fallback if no frontier found)
            int best_x = pose_x;
            int best_y = pose_y;
            
            double min_dist = std::numeric_limits<double>::max();
            int margin = 2; 

            std::cout << "Scanning for reachable frontiers..." << std::endl;

            for(int y = margin; y < max_y - margin; ++y) {
                for(int x = margin; x < max_x - margin; ++x) {
                    
                    // 1. We are looking for a target that is currently FREE (0)
                    if(map.data[x + y * max_x] == 0) { 
                        
                        int unknown_neighbors = 0; 
                        // check neighbors for unknown space 
                        for(int dy = -2; dy <= 2; ++dy) {
                            for(int dx = -2; dx <= 2; ++dx) {
                                if(dx == 0 && dy == 0) continue; // skip self

                                int neighbor_val = map.data[(x + dx) + (y + dy) * max_x];
                                
                                if(neighbor_val == -1) { 
                                    unknown_neighbors += 1;
                                }
                            }
                            if(unknown_neighbors >= 5) break;
                        }

                        // 3. If valid frontier, calculate distance
                        if(unknown_neighbors >= 5) {
                            double dist = std::hypot(x - pose_x, y - pose_y);

                            // 4. Keep the nearest valid frontier that is greater than the min dist
                            if(dist < min_dist && dist > 1/resolution) {
                                min_dist = dist;
                                best_x = x;
                                best_y = y;
                            }
                        }
                    }
                }
            }

            PoseMsg goal;
            goal.position.x = best_x * resolution + og_x;
            goal.position.y = best_y * resolution + og_y;
            
            std::cout << "Best Frontier Found at dist: " << min_dist << std::endl;
            
            return goal;
        } 

        // --- HELPER: Logic to check if an existing goal is still in unknown space ---
        bool is_still_frontier(const OccupancyGrid map, const PoseMsg goal) {
            int gx = (goal.position.x - map.info.origin.position.x) / map.info.resolution;
            int gy = (goal.position.y - map.info.origin.position.y) / map.info.resolution;
            
            // Check a 3x3 window around the goal for ANY unknown pixels (-1)
            for(int dy = -1; dy <= 1; ++dy) {
                for(int dx = -1; dx <= 1; ++dx) {
                    int idx = (gx + dx) + (gy + dy) * map.info.width;
                    if(idx >= 0 && (size_t)idx < map.data.size()) {
                        if(map.data[idx] == -1) return true; 
                    }
                }
            }
            return false;
        }

        NodeStatus tick() override
        {
            Expected<OccupancyGrid> msg
                = getInput<OccupancyGrid>("map");
            // Check if expected is valid. If not, throw its error
            if (!msg)
            {
                throw RuntimeError("missing required input [map]: ", 
                        msg.error() );
            }
            getInput("current_pose",_current_pose);
            OccupancyGrid map=msg.value();
            PoseMsg goal = _find_frontier(map, _current_pose);

            if(_has_active_goal){
                double d_new = std::hypot(goal.position.x - _current_pose.position.x, goal.position.y - _current_pose.position.y);
                double d_cur = std::hypot(_current_active_goal.position.x - _current_pose.position.x, _current_active_goal.position.y - _current_pose.position.y);

                if(d_cur < 1.0){
                    _has_active_goal = false;
                }
                // If current goal is still unknown and not vastly further than the new one, KEEP IT
                else if (is_still_frontier(map, _current_active_goal)) {
                    setOutput("goal_pose", _current_active_goal);
                    return NodeStatus::SUCCESS;
                }
            }
        

        // Otherwise, commit to the new one
        _current_active_goal = goal;
        _has_active_goal = true;
        setOutput("goal_pose", _current_active_goal);
        std::cout << "[FindFrontier] Switching to new goal at: " << _current_active_goal.position.x << ", " << _current_active_goal.position.y << std::endl;
        return NodeStatus::SUCCESS;
    }
};

// ---------------------------------------------------------
// ---------------------------------------------------------
// 1. TagDetection (Debug Version)
// ---------------------------------------------------------

class TagDetection : public RosTopicSubNode<PoseMsg>
{
public:
    TagDetection(const std::string& name,
                 const NodeConfig& conf,
                 const RosNodeParams& params)
        : RosTopicSubNode<PoseMsg>(name, conf, params)
    {}

    static PortsList providedPorts()
    {
        return { OutputPort<PoseMsg>("detected_pose") };
    }

    NodeStatus onTick(const std::shared_ptr<PoseMsg>& msg) override
    {
        auto node_ptr = node_.lock();
        if (!node_ptr) {
            return NodeStatus::FAILURE;
        }

        // 1. Logging for No Data
        if (!msg) {
            RCLCPP_WARN_THROTTLE(node_ptr->get_logger(), *node_ptr->get_clock(), 2000,
                                 "[TagDetection] Waiting for relay node... No pose received yet.");
            return NodeStatus::FAILURE;
        }

        // 2. Logging Success and Coordinates
        RCLCPP_INFO(node_ptr->get_logger(), 
                    "[TagDetection] Data received from relay! Tag at: [x: %.2f, y: %.2f]", 
                    msg->position.x, msg->position.y);

        // 3. Process and Output
        // Note: The relay node already handled the Map transform.
        PoseMsg goal_pose = *msg;
        
        // Example: Offset logic remains here if you want the robot to stop slightly in front
        goal_pose.position.z += 0.5; 

        setOutput("detected_pose", goal_pose);
        
        RCLCPP_DEBUG(node_ptr->get_logger(), "[TagDetection] Blackboard updated with new goal pose.");
        return NodeStatus::SUCCESS;
    }
};


// ---------------------------------------------------------
// NEW TagDetectionTrack: Continuous Background Tracker
// ---------------------------------------------------------
class TagDetectionTrack : public BT::StatefulActionNode
{
public:
    TagDetectionTrack(const std::string& name, const BT::NodeConfiguration& config, rclcpp::Node::SharedPtr node)
        : BT::StatefulActionNode(name, config), node_(node)
    {
        // Set up a standard ROS 2 subscription
        sub_ = node_->create_subscription<PoseMsg>(
            "/track_tag_pose", 10,
            [this](const PoseMsg::SharedPtr msg) {
                last_msg_ = msg;
                has_new_msg_ = true;
            });
    }

    static BT::PortsList providedPorts()
    {
        return { BT::OutputPort<PoseMsg>("track_pose") };
    }

    BT::NodeStatus onStart() override
    {
        // Start running immediately
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        // If we received a new message in the background, update the blackboard
        if (has_new_msg_ && last_msg_) {
            setOutput("track_pose", *last_msg_);
            has_new_msg_ = false; // Reset flag so we don't write unnecessarily
            
            RCLCPP_INFO(node_->get_logger(), 
                "[TagDetectionTrack] Updating track_pose: [x: %.2f, y: %.2f]", 
                last_msg_->position.x, last_msg_->position.y);
        }
        
        // CRITICAL: Always return RUNNING. It will never trigger a FAILURE in the Parallel block.
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        RCLCPP_INFO(node_->get_logger(), "[TagDetectionTrack] Halted.");
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<PoseMsg>::SharedPtr sub_;
    PoseMsg::SharedPtr last_msg_;
    bool has_new_msg_ = false;
};

// ---------------------------------------------------------
// 3. Calc180
// Reads IMU, calculates (Yaw + 180), outputs float
// ---------------------------------------------------------
class Calc180 : public RosTopicSubNode<sensor_msgs::msg::Imu>
{
public:
    Calc180(const std::string& name,
            const NodeConfig& conf,
            const RosNodeParams& params)
        : RosTopicSubNode<sensor_msgs::msg::Imu>(name, conf, params)
    {}

    static PortsList providedPorts()
    {
        // Maps to XML: pose_180="{target_180}"
        return { OutputPort<float>("pose_180") };
    }

    NodeStatus onTick(const std::shared_ptr<sensor_msgs::msg::Imu>& msg) override
    {   
        if (!msg) {
            std::cout << "[Calc180] ERROR: Msg is Null!" << std::endl;
            return NodeStatus::FAILURE;
        }

        // 1. Convert Quaternion to Euler (Yaw)
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        std::cout << "Got Msg";
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // 2. Calculate 180 degrees (PI radians) from current
        float target_yaw = yaw + M_PI;

        // 3. Normalize angle to be between -PI and PI
        if (target_yaw > M_PI) target_yaw -= 2 * M_PI;
        else if (target_yaw < -M_PI) target_yaw += 2 * M_PI;

        // 4. Output the result
        setOutput("pose_180", target_yaw);

        std::cout << "Calc180: Current: " << yaw << "| Target:" << target_yaw;

        return NodeStatus::SUCCESS;
    }
};

// ---------------------------------------------------------
// 4. SetHomePos
// Sets a blackboard variable to 0,0,0
// ---------------------------------------------------------
class SetHomePos : public SyncActionNode
{
public:
    SetHomePos(const std::string& name, const NodeConfig& config)
        : SyncActionNode(name, config)
    {}

    static PortsList providedPorts()
    {
        // Maps to XML: home_pose="{target_pose}"
        return { OutputPort<PoseMsg>("home_pose") };
    }

    NodeStatus tick() override
    {
        PoseMsg home;
        home.position.x = 0.0;
        home.position.y = 0.0;
        home.position.z = 0.0;

        // Orientation 0
        home.orientation.w = 1.0;
        home.orientation.x = 0.0;
        home.orientation.y = 0.0;
        home.orientation.z = 0.0;

        setOutput("home_pose", home);
        std::cout << "[SetHomePos] Home pose set to (0,0,0)" << std::endl;

        return NodeStatus::SUCCESS;
    }
};

// ---------------------------------------------------------
// Condition Node: IsAtPose
// Checks if robot is near a dynamic target pose.
// Inputs: "pose" (PoseMsg)
// ---------------------------------------------------------
class IsAtPose : public BT::ConditionNode
{
public:
    IsAtPose(const std::string& name, const BT::NodeConfiguration& config, 
             std::shared_ptr<tf2_ros::Buffer> tf_buffer)
        : BT::ConditionNode(name, config), tf_buffer_(tf_buffer)
    {}

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<PoseMsg>("pose") };
    }

    BT::NodeStatus tick() override
    {
        PoseMsg target;
        if (!getInput("pose", target)) {
            throw BT::RuntimeError("IsAtPose missing required input [pose]");
        }

        try {
            // 1. Get current robot pose from TF
            geometry_msgs::msg::TransformStamped t = tf_buffer_->lookupTransform(
                "map", "base_link", tf2::TimePointZero, std::chrono::milliseconds(100));

            // 2. Calculate Distance to Target
            double dx = t.transform.translation.x - target.position.x;
            double dy = t.transform.translation.y - target.position.y;
            double dist = std::hypot(dx, dy);

            std::cout << "[IsAtPose] Dist to target: " << dist << "m" << std::endl;

            // 3. Threshold (e.g., 0.5 meters)
            if(dist < 0.5) {
                return BT::NodeStatus::SUCCESS;
            }
            
            return BT::NodeStatus::FAILURE;

        } catch (const tf2::TransformException & ex) {
            return BT::NodeStatus::FAILURE;
        }
    }

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};


// ---------------------------------------------------------
// NEW NODE: TrackTag (Optimized)
// Only publishes if the angle changes significantly
// ---------------------------------------------------------
// ---------------------------------------------------------
// NEW NODE: TrackTag (Map -> Arm Base Frame)
// ---------------------------------------------------------
class TrackTag : public BT::StatefulActionNode
{
public:
    TrackTag(const std::string& name, const BT::NodeConfiguration& config,
             rclcpp::Node::SharedPtr node, std::shared_ptr<tf2_ros::Buffer> tf_buffer)
        : BT::StatefulActionNode(name, config), node_(node), tf_buffer_(tf_buffer)
    {
        publisher_ = node_->create_publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>(
            "set_position", 10);
    }

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<PoseMsg>("tag_pose") };
    }

    NodeStatus onStart() override
    {
        last_angle_ = -999.0;
        return NodeStatus::RUNNING;
    }

    NodeStatus onRunning() override
    {
        PoseMsg tag_pose_map;
        if (!getInput("tag_pose", tag_pose_map)) {
            return NodeStatus::RUNNING;
        }

        try {
            // TRANSFORMATION EXPLAINED:
            // We transform the Tag Position into 'panda_link0' frame.
            // 'panda_link0' is the static base of the arm.
            // This gives us the (X,Y) of the tag as if we were standing on the arm's base.
            geometry_msgs::msg::TransformStamped t_arm_map = tf_buffer_->lookupTransform(
                "panda_link0", "odom", tf2::TimePointZero);

            tf2::Transform tf_arm_map;
            tf2::fromMsg(t_arm_map.transform, tf_arm_map);

            tf2::Vector3 tag_pos_map(tag_pose_map.position.x, tag_pose_map.position.y, tag_pose_map.position.z);
            tf2::Vector3 tag_pos_arm = tf_arm_map * tag_pos_map;

            // Calculate Yaw (Rotation around Z)
            double arm_target_angle = std::atan2(tag_pos_arm.y(), tag_pos_arm.x());

            // DEBUG PRINT: Show the user what angle we are trying to reach
            // std::cout << "[TrackTag] Raw Target: " << arm_target_angle << " rad";

            // HARD LIMIT CHECK
            // The SDF limits shoulder_joint to +/- 2.89 rad.
            // If the tag is straight forward (3.14 rad), we MUST clamp it or the motor rejects it.
            double limit = 2.89; 
            if (arm_target_angle > limit) arm_target_angle = limit;
            if (arm_target_angle < -limit) arm_target_angle = -limit;

            // std::cout << " | Clamped: " << arm_target_angle << std::endl;

            auto message = dynamixel_sdk_custom_interfaces::msg::SetPosition();

            // Init pos
            if(init_pos){
                init_pos = false;
                for(int i = 1; i < 4; i++){
                    message.id = i;
                    message.position = 2048;

                    publisher_->publish(message);
                }
            }
            // Deadband check (0.05 rad is approx 3 degrees)
            if (std::abs(arm_target_angle - last_angle_) > 0.05 && not init_pos) 
            {
                message.id = 1; // Shoulder Joint
                
                // Convert Radians to Raw Value (0-4095)
                // Center (0 rad) is 2048.
                message.position = static_cast<uint32_t>((arm_target_angle * 4096.0 / (2.0 * M_PI)) + 2048.0);
                
                publisher_->publish(message);
                last_angle_ = arm_target_angle;
            }
            
            return NodeStatus::RUNNING;

        } catch (const tf2::TransformException & ex) {
            // This often happens once at startup, which is fine.
            return NodeStatus::RUNNING;
        }
    }

    void onHalted() override
    {
        std::cout << "[TrackTag] Stopped tracking." << std::endl;
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Publisher<dynamixel_sdk_custom_interfaces::msg::SetPosition>::SharedPtr publisher_;
    double last_angle_ = -999.0; 
    bool init_pos = true;
};

static const char* xml_text = R"(
<root BTCPP_format="4" >
     <BehaviorTree ID="MainTree">
        <Sequence>
            <WaitForSeconds    seconds="10"/>
            <SaySomething   message="mission started..." />
            <SaySomething   message="spin started..." />
            <Spin target_yaw="6.283"/>

            <RetryUntilSuccessful num_attempts="1200000">
                <Sequence>
                    <MapService map = "{map}" />
                    <FindFrontier goal_pose = "{target_pose}" 
                        current_pose="{target_pose}"
                        map ="{map}"/>
                    <NavigateToPose pose = "{target_pose}" />

                    <WaitForSeconds    seconds="5"/>
                  <TagDetection detected_pose = "{tag_pose}"/>
                </Sequence>
            </RetryUntilSuccessful>

            <SaySomething   message="April Tag Detected" />
            <RetryUntilSuccessful num_attempts="120">
                <Sequence>
                    <NavigateToPose pose = "{tag_pose}"/>
                    <WaitForSeconds    seconds="5"/>
                    <IsAtPose pose="{tag_pose}" />
                </Sequence>
            </RetryUntilSuccessful>
            <SaySomething   message="Rotate 180" />
            <RetryUntilSuccessful num_attempts="120000">
                <Sequence>
                    <Parallel success_count="1" failure_count="1">
                        <TagDetectionTrack track_pose="{track_pose}"/>
                        <Spin target_yaw="3.14159"/>
                        <TrackTag tag_pose="{track_pose}"/>
                    </Parallel>
                    <SaySomething message="Operating Arm..." />
                    <!-- Wait for a while, this is where the april tag would be picked up by the arm -->
                    <WaitForSeconds    seconds="10"/>
               </Sequence>
            </RetryUntilSuccessful>
            <Sequence>
                <SaySomething message="Returning Home..." />
                <SetHomePos home_pose="{target_pose}"/>

                <RetryUntilSuccessful num_attempts="20">
                    <Sequence>
                        <NavigateToPose pose="{target_pose}" />
                        <WaitForSeconds    seconds="2"/>
                        <IsAtPose pose="{target_pose}" />
                    </Sequence>
                </RetryUntilSuccessful>

                <SaySomething message="mission completed!" />
            </Sequence> 
         </Sequence>
     </BehaviorTree>
 </root>
 )";

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    BT::BehaviorTreeFactory factory;
    // these are the simple BT nodes
    factory.registerNodeType<WaitForSeconds>("WaitForSeconds");
    factory.registerNodeType<SaySomething>("SaySomething");
    factory.registerNodeType<MapFinished>("MapFinished");
    factory.registerNodeType<FindFrontier>("FindFrontier");

    // this is the ROS2 action client node so it needs some extra parameters
    auto node = std::make_shared<rclcpp::Node>("spin_client");
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // provide the ROS node and the name of the action service
    RosNodeParams params; 
    params.nh = node;
    params.default_port_value = "spin";
    factory.registerNodeType<SpinAction>("Spin", params);

    // this is the ROS2 action client node so it needs some extra parameters
    auto nav_node = std::make_shared<rclcpp::Node>("navigate_client");
    // provide the ROS node and the name of the action service
    RosNodeParams nav_params; 
    nav_params.nh = nav_node;
    nav_params.default_port_value = "navigate_to_pose";
    nav_params.server_timeout = std::chrono::milliseconds(1000);
    factory.registerNodeType<NavigateToPoseAction>("NavigateToPose", nav_params);

    auto map_service_node = std::make_shared<rclcpp::Node>("get_map_client");
    // provide the ROS node and the name of the  service
    RosNodeParams map_service_params; 
    map_service_params.nh = map_service_node;
    map_service_params.default_port_value = "slam_toolbox/dynamic_map";
    factory.registerNodeType<MapService>("MapService", map_service_params);
    
    // 1. TagDetection (Subscribes to routing topic)
    RosNodeParams tag_params;
    tag_params.nh = node;
    tag_params.default_port_value = "/nav_tag_pose"; 
    factory.registerNodeType<TagDetection>("TagDetection", tag_params);

    // 2. TagDetection for Tracking (Odom frame)
    // 2. TagDetection for Tracking (Custom background node)
    factory.registerBuilder<TagDetectionTrack>("TagDetectionTrack",
        [node](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<TagDetectionTrack>(name, config, node);
        });

    // 2. Calc180 (Subscribes to IMU)
    RosNodeParams imu_params;
    imu_params.nh = node; // Uses the main spin_client node
    imu_params.default_port_value = "/imu"; // 
    factory.registerNodeType<Calc180>("Calc180", imu_params);

    // 3. SetHomePos (Simple Action, no ROS params needed)
    factory.registerNodeType<SetHomePos>("SetHomePos");

    factory.registerBuilder<IsAtPose>("IsAtPose",
        [tf_buffer](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<IsAtPose>(name, config, tf_buffer);
        });

    factory.registerBuilder<TrackTag>("TrackTag",
        [node, tf_buffer](const std::string& name, const BT::NodeConfiguration& config) {
            return std::make_unique<TrackTag>(name, config, node, tf_buffer);
        });

    auto tree = factory.createTreeFromText(xml_text);
 
    // Here, instead of tree.tickWhileRunning(),
    // we prefer our own loop.
    //    std::cout << "--- ticking\n";
    NodeStatus status = tree.tickOnce();
    //    std::cout << "--- status: " << toStr(status) << "\n\n";

    while(status == NodeStatus::RUNNING) 
    {
        rclcpp::spin_some(node);
        // Sleep to avoid busy loops.
        // do NOT use other sleep functions!
        // Small sleep time is OK, here we use a large one only to
        // have less messages on the console.
        tree.sleep(std::chrono::milliseconds(100));

        //       std::cout << "--- ticking\n";
        status = tree.tickOnce();
        //        std::cout << "--- status: " << toStr(status) << "\n\n";
    }
    return 0;
}
