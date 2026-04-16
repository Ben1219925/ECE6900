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
        PoseMsg _find_frontier(OccupancyGrid map, PoseMsg pose) 
{
    auto resolution = map.info.resolution; 
    auto og_x = map.info.origin.position.x;
    auto og_y = map.info.origin.position.y;
    int width = map.info.width;
    int height = map.info.height;
    
    unsigned pose_x = (pose.position.x - og_x) / resolution;
    unsigned pose_y = (pose.position.y - og_y) / resolution;
    
    std::vector<bool> visited(width * height, false);
    struct Point { int x, y; };
    std::vector<std::vector<Point>> clusters;

    // 1. Identify all frontier pixels and group them into clusters
    for(int y = 1; y < height - 1; ++y) {
        for(int x = 1; x < width - 1; ++x) {
            int idx = x + y * width;

            // Only start a cluster if it's unknown, not visited, and touches free space
            if(!visited[idx] && map.data[idx] == -1) {
                
                bool is_frontier = false;
                for(int dy = -1; dy <= 1; ++dy) {
                    for(int dx = -1; dx <= 1; ++dx) {
                        if(map.data[(x + dx) + (y + dy) * width] == 0) {
                            is_frontier = true;
                            break;
                        }
                    }
                    if(is_frontier) break;
                }

                if(is_frontier) {
                    // Start BFS to find all connected frontier pixels
                    std::vector<Point> current_cluster;
                    std::vector<Point> q;
                    q.push_back({x, y});
                    visited[idx] = true;

                    int head = 0;
                    while(head < q.size()){
                        Point p = q[head++];
                        current_cluster.push_back(p);

                        for(int dy = -1; dy <= 1; ++dy) {
                            for(int dx = -1; dx <= 1; ++dx) {
                                int nx = p.x + dx;
                                int ny = p.y + dy;
                                int n_idx = nx + ny * width;

                                if(nx >= 0 && nx < width && ny >= 0 && ny < height && 
                                   !visited[n_idx] && map.data[n_idx] == -1) {
                                    
                                    // Check if neighbor is also a frontier pixel
                                    bool n_is_frontier = false;
                                    for(int n_dy = -1; n_dy <= 1; ++n_dy) {
                                        for(int n_dx = -1; n_dx <= 1; ++n_dx) {
                                            if(map.data[(nx+n_dx) + (ny+n_dy)*width] == 0) {
                                                n_is_frontier = true; break;
                                            }
                                        }
                                        if(n_is_frontier) break;
                                    }

                                    if(n_is_frontier) {
                                        visited[n_idx] = true;
                                        q.push_back({nx, ny});
                                    }
                                }
                            }
                        }
                    }
                    // Filter out tiny clusters (noise)
                    if(current_cluster.size() > 5) { 
                        clusters.push_back(current_cluster);
                    }
                }
            }
        }
    }

    // 2. Find the best cluster (the closest one + 10 to ensure that the robot always moves)
    Point best_point = {(int)pose_x, (int)pose_y};
    int biggest_cluster = 0;
    double max_dist = -1.0;

    for(const auto& cluster : clusters)     {
        // Calculate Centroid of the cluster
        double sum_x = 0, sum_y = 0;
        for(const auto& p : cluster) {
            sum_x += p.x;
            sum_y += p.y;
        }
        int cx = sum_x / cluster.size();
        int cy = sum_y / cluster.size();       

        // Calculate Euclidean distance from current pose to centroid
        double dx = cx - pose_x;
        double dy = cy - pose_y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        if(distance > max_dist){
            biggest_cluster = cluster.size();
            best_point = {cx, cy};
        }
    
}

    PoseMsg goal;
    goal.position.x = best_point.x * resolution + og_x;
    goal.position.y = best_point.y * resolution + og_y;
    
    std::cout << "Found " << clusters.size() << std::endl;
    
    //if(min_dist == 0){ 
    //    goal.position.x = 0;
    //    goal.position.y = 0;
    //}

    return goal;
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
            setOutput("goal_pose", goal);
            return NodeStatus::SUCCESS;          
        }
};

// ---------------------------------------------------------
// ---------------------------------------------------------
// 1. TagDetection (Debug Version)
// ---------------------------------------------------------
using AprilTagArray = apriltag_msgs::msg::AprilTagDetectionArray;

class TagDetection : public RosTopicSubNode<AprilTagArray>
{
public:
    TagDetection(const std::string& name,
                 const NodeConfig& conf,
                 const RosNodeParams& params)
        : RosTopicSubNode<AprilTagArray>(name, conf, params)
    {
        // 1. Safe Initialization
        auto node = params.nh.lock();
        if(!node) {
            throw RuntimeError("TagDetection: Node is null in constructor!");
        }
        
        // Initialize TF
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        std::cout << "[TagDetection] Initialized and Listener created." << std::endl;
    }

    static PortsList providedPorts()
    {
        // Maps to XML: detected_pose="{tag_pose}"
        return { OutputPort<PoseMsg>("detected_pose") };
    }

    NodeStatus onTick(const std::shared_ptr<AprilTagArray>& msg) override
    {
        // Debug A: Confirm we entered tick
        std::cout << "[TagDetection] TICKED. Msg Addr: " << msg.get() << std::endl;

        if (!msg) {
            std::cout << "[TagDetection] ERROR: Msg is Null!" << std::endl;
            return NodeStatus::FAILURE;
        }

        // Debug B: Check vector size
        std::cout << "[TagDetection] Checking detections size: " << msg->detections.size() << std::endl;

        if(msg->detections.empty())
        {
            std::cout << "[TagDetection] No tags seen. Returning FAILURE." << std::endl;
            return NodeStatus::FAILURE;
        }

        // Debug C: Vector is safe, accessing [0]
        auto detection = msg->detections[0];
        std::string tag_frame = detection.family + ":" + std::to_string(detection.id);

        try {
            // 1. Look up transform from Map -> Tag
            // We want the transform that gives us the Tag's position in the Map frame
            geometry_msgs::msg::TransformStamped t_map_tag = tf_buffer_->lookupTransform(
                "map", tag_frame, tf2::TimePointZero, std::chrono::milliseconds(100));

            // 2. Create the offset in the TAG'S frame
            // AprilTags usually have Z pointing OUT of the paper.
            // So we want to be at (0, 0, 0.5) in the TAG frame.
            tf2::Vector3 offset_in_tag_frame(0.0, 0.0, 0.5); 

            // 3. Rotate this offset into the MAP frame
            // We apply the Tag's orientation to our offset vector
            tf2::Quaternion q_tag_map(
                t_map_tag.transform.rotation.x,
                t_map_tag.transform.rotation.y,
                t_map_tag.transform.rotation.z,
                t_map_tag.transform.rotation.w);
            
            // Rotate the offset vector by the tag's orientation quaternion
            tf2::Vector3 offset_in_map = tf2::quatRotate(q_tag_map, offset_in_tag_frame);

            // 4. Add the rotated offset to the Tag's position
            PoseMsg goal_pose;
            goal_pose.position.x = t_map_tag.transform.translation.x + offset_in_map.x();
            goal_pose.position.y = t_map_tag.transform.translation.y + offset_in_map.y();
            goal_pose.position.z = t_map_tag.transform.translation.z + offset_in_map.z();

            // 5. Orientation: The robot should FACE the tag
            // The tag faces Z+. To look AT the tag, the robot (X+) needs to face Tag's Z-.
            // This is a bit complex, simpler is to keep the Tag's orientation but rotated 180 deg?
            // OR: Just keep the tag's orientation. If the tag is on a wall, 
            // the robot will align with the wall.
            goal_pose.orientation = t_map_tag.transform.rotation;

            // Update Blackboard
            setOutput("detected_pose", goal_pose);
            
            std::cout << "[TagDetection] Goal set 0.5m in front of tag." << std::endl;
            return NodeStatus::SUCCESS;

        } catch (const tf2::TransformException & ex) {
            return NodeStatus::FAILURE;
        }
    }
private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
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
            // Note: Since you offset the tag goal by 0.5m, using 0.5m tolerance is safe.
            if(dist < 0.25) {
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
            return NodeStatus::FAILURE;
        }

        try {
            // TRANSFORMATION EXPLAINED:
            // We transform the Tag Position into 'panda_link0' frame.
            // 'panda_link0' is the static base of the arm.
            // This gives us the (X,Y) of the tag as if we were standing on the arm's base.
            geometry_msgs::msg::TransformStamped t_arm_map = tf_buffer_->lookupTransform(
                "panda_link0", "map", tf2::TimePointZero);

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

                    <!--<WaitForSeconds    seconds="2"/>-->
                  <TagDetection detected_pose = "{tag_pose}"/>
                </Sequence>
            </RetryUntilSuccessful>

            <SaySomething   message="April Tag Detected" />
            <RetryUntilSuccessful num_attempts="120">
                <Sequence>
                    <NavigateToPose pose = "{tag_pose}"/>
                    <WaitForSeconds    seconds="2"/>
                    <IsAtPose pose="{tag_pose}" />
                </Sequence>
            </RetryUntilSuccessful>
            
            <SaySomething   message="Rotate 180" />
            <RetryUntilSuccessful num_attempts="120">
                <Sequence>  
                    <Parallel success_count="1" failure_count="1">
                        <Spin target_yaw="3.14159"/>
                        
                        <TrackTag tag_pose="{tag_pose}"/>
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
    
    // 1. TagDetection (Subscribes to AprilTag topic)
    RosNodeParams tag_params;
    tag_params.nh = node;
    tag_params.default_port_value = "camera2/detections"; 
    factory.registerNodeType<TagDetection>("TagDetection", tag_params);

    // 2. Calc180 (Subscribes to IMU)
    RosNodeParams imu_params;
    imu_params.nh = node; // Uses the main spin_client node
    imu_params.default_port_value = "/imu"; // <--- VERIFY THIS TOPIC NAME
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
