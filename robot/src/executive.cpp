#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/bt_service_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/compute_path_to_pose.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "ros_interfaces/srv/get_pose.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace chr = std::chrono;
using namespace BT;
using PoseMsg = geometry_msgs::msg::Pose;
using Point = geometry_msgs::msg::Point;
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

// Simple function that return a NodeStatus
NodeStatus CheckBattery()
{
  std::cout << "[ Battery: OK ]" << std::endl;
  return NodeStatus::SUCCESS;
}

// SyncActionNode (synchronous action) with an input port.
/*
   a SyncActionNode can only return SUCCESS or FAILURE
   it cannot return RUNNING. so it is only good for simple, fast operations
 */
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
class Wait : public StatefulActionNode
{
  public:
    Wait(const std::string& name, const NodeConfiguration& config)
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
NodeStatus Wait::onStart()
{
  if ( !getInput<unsigned>("seconds", _seconds))
  {
    throw RuntimeError("missing required input [seconds]");
  }
  printf("[ Wait: ] seconds = %d\n",_seconds);
  _completion_time = chr::system_clock::now() + chr::milliseconds(_seconds*1000);
  return NodeStatus::RUNNING;
}

NodeStatus Wait::onRunning()
{
  std::this_thread::sleep_for(chr::milliseconds(1000));
  if(chr::system_clock::now() >= _completion_time)
  {
    std::cout << "[ Wait: FINISHED ]" << std::endl;
    return NodeStatus::SUCCESS;
  }
  return NodeStatus::RUNNING;
}
void Wait::onHalted()
{
  printf("[ Wait: ABORTED ]");
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
      std::cout << "spinning result "  << wr.result->error_msg << "\n";
      return NodeStatus::SUCCESS;
    }
    virtual NodeStatus onFailure(ActionNodeErrorCode error) override
    {
      std::cout << "spinning " << error << "\n";
      return NodeStatus::FAILURE;
    }
    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
    {
      std::cout << "spinning feedback " << feedback->angular_distance_traveled << "\n";
      return NodeStatus::RUNNING;
    }
};
using GetPose = ros_interfaces::srv::GetPose;
class PoseService: public RosServiceNode<GetPose>
{
  public:
    PoseService(const std::string& name,
        const NodeConfig& conf,
        const RosNodeParams& params)
      : RosServiceNode<GetPose>(name, conf, params)
    {}
    static PortsList providedPorts()
    {
      return providedBasicPorts({OutputPort<PoseMsg>("pose")});
    }
    bool setRequest(Request::SharedPtr& request) override
    {
      (void)request;  // don't do anything
      return true;
    }
    NodeStatus onResponseReceived(const Response::SharedPtr& response) override
    {
      std::cout << "Got a pose: " << response->pose.position.x << ", " << response->pose.position.y<<"\n";
      setOutput("pose",response->pose);
      return NodeStatus::SUCCESS;
    }
    virtual NodeStatus onFailure(ServiceNodeErrorCode error) override
    {
      std::cerr << "PoseService onFailure error code: "<< error << std::endl;
      return NodeStatus::FAILURE;
    }
};
int getPixel(OccupancyGrid &map, int x, int y) {
  return map.data[x+map.info.width*+y];
}
bool isNearWall(OccupancyGrid &map, int x, int y) {
  unsigned max_x=map.info.width;
  unsigned max_y=map.info.height;
  int wall_search_size=10;
  for(int _x=x-wall_search_size;_x<=x+wall_search_size;++_x) {
    for(int _y=y-wall_search_size;_y<=y+wall_search_size;++_y) {
      if(_x<0 || _x>(int)max_x || _y<0 || _y>(int)max_y) {
        continue;
      }
      if(getPixel(map,_x,_y)>0) {
        return true;
      }
    }
  }
  return false;
}
bool isNearKnown(OccupancyGrid &map, int x, int y) {
  unsigned max_x=map.info.width;
  unsigned max_y=map.info.height;
  int search_size=1;
  for(int _x=x-search_size;_x<=x+search_size;++_x) {
    for(int _y=y-search_size;_y<=y+search_size;++_y) {
      if(_x<0 || _x>(int)max_x || _y<0 || _y>(int)max_y) {
        continue;
      }
      if(getPixel(map,_x,_y)==0) {
        return true;
      }
    }
  }
  return false;
}
// check if a pixel is a frontier
// a frontier is an unknown pixel with at least one
// adjacent known clear pixel
bool isFrontier(OccupancyGrid &map, int x, int y) {
  //            std::cout << "checking for frontier "<<x<<" "<<y<<" "<<max_x<<" " <<map.info.height<<"\n";
  // not a frontier if not unknown
  if(getPixel(map,x,y) != -1) {
    //                std::cout << "known "<<x<<" "<<y<<" "<<getPixel(map,x,y)<<"\n";
    return false;
  }
  //std::cout << "unknown "<<x<<" "<<y<<" "<<getPixel(map,x,y)<<"\n";
  // ignore points close to the wall
  if(isNearWall(map,x,y)) {
    return false;
  }
  // check all pixels around it and return true if any are known clear
  if(isNearKnown(map,x,y)) {
    //std::cout << "found frontier !!!"<<x<<" "<<y<<" "<<max_x<<" " <<map.info.height<<"\n";
    return true;
  }
  return false;
} 
bool isIsland(OccupancyGrid &map, int x, int y) {
  float dist_m = 0.5; // meters min area to pursue
  unsigned dist = dist_m/map.info.resolution;
  unsigned min_neighbors= 3;
  // count how many pixels square around it are unknown
  unsigned cnt=0;
  for(int col=x-dist;col<=(int)(x+dist);col++) {
    for(int row=y-dist;row<=(int)(y+dist);row++) {
      if(getPixel(map,col,row)==-1) {
        cnt++;
      }
    }
  }
  if(cnt<min_neighbors) {
    std::cout << "rejected because island " << x<<","<<y<<" dist: "<<dist<<" cnt= " << cnt <<"\n";
    return false;
  } else {
    return true;
  }
}
float getAbsRelHeading(float a, float b) {
  float x = abs(a-b);
  x = x > M_PI ? M_PI*2-x : x;
  return abs(x);
}

bool find_frontier(OccupancyGrid map, PoseMsg pose, PoseMsg &goal)
{
  bool foundGoal = false;
  unsigned x,y;
  unsigned pose_x=(pose.position.x-map.info.origin.position.x)/map.info.resolution;
  unsigned pose_y=(pose.position.y-map.info.origin.position.y)/map.info.resolution;
  float yaw=tf2::getYaw(pose.orientation);
  unsigned max_x=map.info.width;
  unsigned max_y=map.info.height;
  float r, heading;
  unsigned best_x=0,best_y=0;
  float  margin_m = 0.5; // meters from base_link to edge of map to ignore
  unsigned margin = margin_m/map.info.resolution; // pixels from base_link to ignore
  float max_r_m = 15; // meters from base link to ignore (too far away)
  unsigned max_r = max_r_m/map.info.resolution; // pixels from base_link to ignore
  float min_r_m = 2; // meters from base link to ignore (too close)
  unsigned min_r = min_r_m/map.info.resolution; // pixels from base_link to ignore
  float best_heading=1000; // heading to the best frontier
                           //best_r=0.0;
  std::cout<<"FindFrontier:\n";
  std::cout<<"Map info: origin:"<< map.info.origin.position.x << ", "
    << map.info.origin.position.x <<" size: "<<max_x<<","<<max_y<< std::endl;
  std::cout<<"My pose : "<<pose.position.x<<","<<pose.position.y<<" yaw: "<<yaw<<std::endl;
  std::cout<<"My pose in pixels: "<<pose_x<<","<<pose_y<<" yaw: "<<yaw<<std::endl;
  for(x=0;x<max_x;++x) {
    for(y=0;y<max_y;++y) {  // for every point on the map
                            // check too close to edges
      if( (x<margin) || (x>max_x-margin) || 
          (y<margin) || (y>max_y-margin)) {
        continue;
      }
      // check that it is a frontier pixel
      if(!isFrontier(map, x,y)) {
        continue;
      }
      if(!isIsland(map, x,y)) {
        continue;
      }
      // calculate the distance to it
      r=sqrt(((float)x-(float)pose_x)*((float)x-(float)pose_x)+((float)y-(float)pose_y)*((float)y-(float)pose_y));

      // calculate the heading from the robot to the pixel
      heading = atan2(((float)y-(float)pose_y),((float)x-(float)pose_x));
      float rel_heading = getAbsRelHeading(yaw,heading);
      if( (r>min_r) && (r<max_r) && (rel_heading<best_heading) ) {
        std::cout << "new best frontier! x: "<<x<<" y: "<<y<<" r: "<<r<<" heading: "<<heading<<
          " prev_x: "<<best_x<<" prev_y: "<<best_y<<" rel_head: "<< rel_heading<<" prev_heading: "<<best_heading<<"\n";
        //best_r=r;
        best_x=x;
        best_y=y;
        best_heading=rel_heading;
        foundGoal=true;
      }
    }
  }
  if(foundGoal) {
    goal.position.x=best_x*map.info.resolution+map.info.origin.position.x;
    goal.position.y=best_y*map.info.resolution+map.info.origin.position.y;
    std::cout<<"Found goal pose : "<<goal.position.x<<","<<goal.position.y<<" heading: "<<heading<<std::endl;
    return true;
  } else {
    std::cout<<"no frontier found!!!!"<<std::endl;
    return false;
  }
}

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
      PoseMsg current_pose;
      getInput("current_pose",current_pose);
      OccupancyGrid map=msg.value();
      PoseMsg goal;
      if(find_frontier(map, current_pose, goal)) {
        setOutput("goal_pose", goal);
        return NodeStatus::SUCCESS;
      } else {
        return NodeStatus::FAILURE;
      }
    }
};
using ComputePathToPose = nav2_msgs::action::ComputePathToPose;
using Path = nav_msgs::msg::Path;
class ComputePathToPoseAction: public RosActionNode<ComputePathToPose>
{
  public:
    ComputePathToPoseAction(const std::string& name,
        const NodeConfig& conf,
        const RosNodeParams& params)
      : RosActionNode<ComputePathToPose>(name, conf, params)
    {}
    static PortsList providedPorts()
    {
      return {
        InputPort<PoseMsg>("goal_pose"),
          InputPort<PoseMsg>("start_pose"),
          OutputPort<uint16_t>("error_code"),
          OutputPort<Path>("path")
      };
    }
    bool setGoal(RosActionNode::Goal& goal) override 
    {
      getInput("goal_pose", goal.goal.pose);
      getInput("start_pose", goal.start.pose);
      goal.goal.header.frame_id="map";
      goal.start.header.frame_id="map";
      return true;
    }
    NodeStatus onResultReceived(const WrappedResult& wr) override
    {
      std::cout << "ComputePathToPose result " << wr.result->error_code << " " << wr.result->error_msg << "\n";
      setOutput("path", wr.result->path);
      return NodeStatus::SUCCESS;
    }
    virtual NodeStatus onFailure(ActionNodeErrorCode error) override
    {
      std::cerr << "ComputePathToPoseAction onFailure error code: "<< error << std::endl;
      return NodeStatus::FAILURE;
    }
    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
    {
      std::cout<< "ComputePathToPose feedback Recieved:" <<  feedback <<"\n";
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
      //RCLCPP_INFO(node_->get_logger(), "NavigateToPose Goal reached!");
      std::cout << "NavigateToPose Goal reached! " << wr.result << "\n";
      return NodeStatus::SUCCESS;
    }
    virtual NodeStatus onFailure(ActionNodeErrorCode error) override
    {
      // for some reason, it seems that nav to pose always returns fail?
      // it times out as a known bug...
      // https://github.com/BehaviorTree/BehaviorTree.ROS2/issues/76
      std::cout<< "NavigateToPose called onFailure even though it shouldn't!:" <<  error <<"\n";
      // should return fail, but then the tree aborts.
      //return NodeStatus::FAILURE;
      // so return success and figure it out in the behavours
      // with delays?
      return NodeStatus::SUCCESS;
    }
    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
    {
      (void)feedback; // unused
                      //std::cout<< "Nav to pose feedback Recieved:" <<  feedback <<"\n";
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
      return providedBasicPorts({OutputPort<nav_msgs::msg::OccupancyGrid>("map")});
    }
    bool setRequest(Request::SharedPtr& request) override
    {
      (void)request;  // unused
      return true;
    }
    NodeStatus onResponseReceived(const Response::SharedPtr& response) override
    {
      setOutput("map",response->map);
      return NodeStatus::SUCCESS;
    }
    virtual NodeStatus onFailure(ServiceNodeErrorCode error) override
    {
      std::cerr << "MapService onFailure error code: "<< error << std::endl;
      return NodeStatus::FAILURE;
    }
};
static const char* xml_text = R"(
<root BTCPP_format="4" >
     <BehaviorTree ID="MainTree">
        <Sequence>
            <Wait seconds="10"/>
            <SaySomething   message="mission started..." />
            <SaySomething   message="mission completed!" />
        </Sequence>
     </BehaviorTree>
 </root>
 )";
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  BT::BehaviorTreeFactory factory;
  // these are the simple BT nodes they don't interract with ROS
  factory.registerSimpleCondition("BatteryOK", std::bind(CheckBattery));
  factory.registerNodeType<Wait>("Wait");
  factory.registerNodeType<SaySomething>("SaySomething");

  // this is the ROS2 action client node 
  auto node = std::make_shared<rclcpp::Node>("spin_client");
  // provide the ROS node and the name of the action service
  RosNodeParams params; 
  params.nh = node;
  params.default_port_value = "spin";
  factory.registerNodeType<SpinAction>("Spin", params);

  // this is the ROS2 service client node 
  auto map_service_node = std::make_shared<rclcpp::Node>("get_map_client");
  // provide the ROS node and the name of the  service
  RosNodeParams map_service_params; 
  map_service_params.nh = map_service_node;
  map_service_params.default_port_value = "slam_toolbox/dynamic_map";
  factory.registerNodeType<MapService>("MapService", map_service_params);

  auto tree = factory.createTreeFromText(xml_text);

  //  start the behaviour tree
  NodeStatus status = tree.tickOnce();

  while(status == NodeStatus::RUNNING) 
  {
    tree.sleep(std::chrono::milliseconds(100));
    status = tree.tickOnce();
  }
  return 0;
}


