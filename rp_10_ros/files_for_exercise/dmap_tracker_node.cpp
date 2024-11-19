#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <rp_stuff/grid_map.h>
#include <rp_stuff/dmap.h>
#include <rp_stuff/draw_helpers.h>
#include <rp_stuff/dmap_localizer.h>

using namespace std;


DMap dmap(0,0);
GridMapping grid_mapping;
DMapLocalizer localizer;

// parameters
float resolution=0.05;
float max_range=10;
float expansion_range=1;
Canvas canvas;
Isometry2f lmap_pose;
bool first_scan=true;

// TODO 1: implement this function
// dest contains the valid endpoints of the scan in world coordinates
void computeScanEndpoints(std::vector<Vector2f>& dest, const sensor_msgs::LaserScan& scan) {
}

// TODO 2: implement this function
// dest contains the grid integer endpoints computed from the
// world endpoints in src
void computeGridEndpoints(std::vector<Vector2i>& dest, const std::vector<Vector2f>& src) {
}

// TODO 3: implement this function
// it initializes the localized based on what done in dmap_localize_node
void initLocalizer(std::vector<Vector2f>& scan_endpoints) {
}

void laserCallback(const sensor_msgs::LaserScan& scan) {
  std::vector<Vector2f> scan_endpoints;
  computeScanEndpoints(scan_endpoints, scan);
  
  if (first_scan) {
    initLocalizer(scan_endpoints);
    first_scan=false;
  } else {
    float angle=scan.angle_min;
    localizer.localize(scan_endpoints,10);
    float translation_norm=localizer.X.translation().norm();
    float orientation_norm=fabs(Eigen::Rotation2Df(localizer.X.linear()).angle());
    if (translation_norm>0.5 || orientation_norm>0.5) {
      lmap_pose = lmap_pose*localizer.X;
      initLocalizer(scan_endpoints);
    }
  }
  localizer.distances.draw(canvas, true);
  for (const auto& ep: scan_endpoints) {
    drawCircle(canvas, grid_mapping.world2grid(localizer.X*ep), 3, 255);
  }
  showCanvas(canvas,1);

  Isometry2f global_pose=lmap_pose*localizer.X;
  cerr << global_pose.translation().transpose()
       << " " << Eigen::Rotation2Df(global_pose.linear()).angle() << endl;
}

int main(int argc, char** argv) {
  ros:: init(argc, argv, "boh");
  ros::NodeHandle n;
  string topic_name=argv[1];

  // compute the grid size
  int grid_size = 2*(max_range+expansion_range)/resolution;
  dmap.resize(grid_size, grid_size);
  grid_mapping.reset(Vector2f(-grid_size*resolution/2, grid_size*resolution/2), resolution);
  cerr << "grid_size" << grid_size << endl;
  cerr << "world center"  << grid_mapping.world2grid(Vector2f(0,0)).transpose() << endl;

  lmap_pose.setIdentity();
  
  ros::Subscriber sub = n.subscribe<const sensor_msgs::LaserScan&>(topic_name, 10, laserCallback);
  while (ros::ok()) {
    ros::spinOnce();
  }
}
