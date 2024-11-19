#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <rp_stuff/grid_map.h>
#include <rp_stuff/dmap.h>
#include <rp_stuff/draw_helpers.h>
#include <rp_stuff/dmap_localizer.h>

using namespace std;


GridMapping grid_mapping;
// here we add a localizer
DMapLocalizer localizer;

// parameters
float resolution=0.05;
float max_range=10;
float expansion_range=1;
Canvas canvas;

bool first_scan=true;
void laserCallback(const sensor_msgs::LaserScan& scan) {
  std::vector<Vector2f> scan_endpoints;

  // TODO 1: scan_endpoints shouold contain the
  //         valid endpoints of the scan in world coordinates
  
  if (first_scan) {
    DMap dmap(0,0);
    // TODO 2: construct the dmap with the grid endpoints (see dmap_show_live_node.cpp
    
    Grid_<float> distances;
    dmap.copyTo(distances);
    for (auto& d: distances.cells) {
      d*=resolution;
    }
    localizer.setMap(grid_mapping, distances);
    first_scan=false;
    localizer.X.setIdentity();
  } else {
    float angle=scan.angle_min;
    localizer.localize(scan_endpoints,10);
  }
  localizer.distances.draw(canvas, true);
  for (const auto& ep: scan_endpoints) {
    drawCircle(canvas, grid_mapping.world2grid(localizer.X*ep), 3, 255);
  }
  showCanvas(canvas,1);
}

int main(int argc, char** argv) {
  ros:: init(argc, argv, "boh");
  ros::NodeHandle n;
  string topic_name=argv[1];
  
  // variables for generating dmap

  // compute the grid size
  int grid_size = 2*(max_range+expansion_range)/resolution;
  dmap.resize(grid_size, grid_size);
  grid_mapping.reset(Vector2f(-grid_size*resolution/2, grid_size*resolution/2), resolution);
  cerr << "grid_size" << grid_size << endl;
  cerr << "world center"  << grid_mapping.world2grid(Vector2f(0,0)).transpose() << endl;

  //TODO 3:
  //subscribe to the callback and spin ROS

}
