#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include "astar_planner.hpp"
#include "map_loader.hpp"

class Planner : public rclcpp::Node {
 public:
  Planner() : Node("planner") {
    auto map = declare_parameter<std::string>("map_file", "");
    auto start = declare_parameter<std::vector<double>>("start", {2.,3.,1.5});
    auto goal = declare_parameter<std::vector<double>>("goal", {18.,16.,1.5});
    double clearance = declare_parameter("clearance", 1.9);
    auto qos = rclcpp::QoS(1).reliable().transient_local();
    pub_ = create_publisher<nav_msgs::msg::Path>("/swarm/path",qos);
    status_ = create_publisher<std_msgs::msg::String>("/swarm/planner_status",qos);
    try {
      if (start.size()!=3 || goal.size()!=3 || clearance < 1.9 || !std::isfinite(clearance))
        throw std::runtime_error("start/goal need 3 coordinates; clearance must be >= 1.9 m for this formation");
      auto [obstacles,bounds] = sim::load_from_json(map);
      for (int i=0;i<3;++i) {
        auto lo=sim::to_array(bounds[0]), hi=sim::to_array(bounds[1]);
        if (!std::isfinite(start[i]) || !std::isfinite(goal[i]) || start[i]<lo[i] || goal[i]<lo[i] || start[i]>hi[i] || goal[i]>hi[i])
          throw std::runtime_error("start/goal outside map");
      }
      if (std::abs(start[2]-goal[2])>1e-6) throw std::runtime_error("fixed-altitude formation requires matching start and goal z");
      if (start[2]<0.6) throw std::runtime_error("flight altitude must be >= 0.6 m");
      for (double dy : {0.,-1.,1.}) {
        for (double z=0.1; z<=start[2]; z+=0.02) {
          if (obstacles.signed_distance({start[0],start[1]+dy,z}) < 0.46)
            throw std::runtime_error("takeoff column intersects obstacle");
        }
      }
      // Formation envelope: 1 m offset + 0.453 m hull radius + tracking allowance.
      auto base = sim::OccupancyGrid::from_obstacles(obstacles,
          {bounds[0].x,bounds[0].y,start[2]}, {bounds[1].x-bounds[0].x,bounds[1].y-bounds[0].y,0},0.25);
      sim::SDFAwareGrid grid(base,obstacles,clearance);
      for (int y=0;y<grid.ny;++y) for (int x=0;x<grid.nx;++x) {
        auto v=grid.index_to_world(x,y,0);
        if (v.x<bounds[0].x+clearance || v.x>bounds[1].x-clearance || v.y<bounds[0].y+clearance || v.y>bounds[1].y-clearance)
          grid.data[y*grid.nx+x]=1;
      }
      auto result = sim::astar_plan(sim::from_array3({start[0],start[1],start[2]}),sim::from_array3({goal[0],goal[1],goal[2]}),grid);
      if (!result.success) throw std::runtime_error("no collision-free formation route");
      result.path.insert(result.path.begin(),{start[0],start[1],start[2]});
      result.path.push_back({goal[0],goal[1],goal[2]});
      // Validate continuous segments: legacy A* uses 26-connected neighbors.
      for (size_t i=1;i<result.path.size();++i) {
        auto a=result.path[i-1],b=result.path[i];
        int n=std::max(1,static_cast<int>(std::ceil(sim::norm(b-a)/0.02)));
        for(int j=0;j<=n;++j) if(obstacles.signed_distance(a+(b-a)*(double(j)/n))<clearance)
          throw std::runtime_error("A* segment clips inflated obstacle; increase clearance or change goal");
      }
      nav_msgs::msg::Path path; path.header.frame_id="world"; path.header.stamp=now();
      for(const auto &v:result.path) {
        geometry_msgs::msg::PoseStamped p; p.header=path.header;
        p.pose.position.x=v.x; p.pose.position.y=v.y; p.pose.position.z=v.z; p.pose.orientation.w=1;
        path.poses.push_back(p);
      }
      pub_->publish(path); std_msgs::msg::String status; status.data="READY"; status_->publish(status);
      RCLCPP_INFO(get_logger(),"Planned %zu formation waypoints",path.poses.size());
    } catch(const std::exception &e) {
      std_msgs::msg::String status; status.data=std::string("FAILED: ")+e.what(); status_->publish(status);
      RCLCPP_ERROR(get_logger(),"%s",status.data.c_str());
    }
  }
 private:
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv); rclcpp::spin(std::make_shared<Planner>()); rclcpp::shutdown();
}
