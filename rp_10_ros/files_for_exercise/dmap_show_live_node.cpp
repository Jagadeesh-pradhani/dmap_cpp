#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <rp_stuff/grid_map.h>
#include <rp_stuff/dmap.h>
#include <rp_stuff/draw_helpers.h>

using namespace std;


// global distance map
DMap dmap(0,0);

// this class offers the world2grid and grid2world facilities
GridMapping grid_mapping; 

// global parameters accessible by the callback
float resolution=0.05;
float max_range=10;
float expansion_range=1;
Canvas canvas;


void laserCallback(const sensor_msgs::LaserScan& scan) {
  std::vector<Vector2i> endpoints;

  //TODO1: populate gthe endpoints vector with the grid endpopints
  //       obtained by rendering the scan
  //       skip the ranges that have range smaller than range_min and
  //       larger than range_max
  
  dmap.clear();
  int dmax2=pow(expansion_range/resolution, 2);

  int ops=dmap.compute(endpoints, dmax2);

  // extract the distances from the dmap, and plot them 
  Grid_<float> distances;
  dmap.copyTo(distances);
  distances.draw(canvas, true);
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


  // TODO 2: subscribe the LaserScan whose topic is stored in "topic_name"
  // to call the laserCallback;

  // TODO 3: run ros
  
}
