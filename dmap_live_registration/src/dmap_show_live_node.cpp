#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <rp_stuff/grid_map.h>
#include <rp_stuff/dmap.h>
#include <rp_stuff/draw_helpers.h>
#include <nav_msgs/OccupancyGrid.h>


using namespace std;

// Global variables
DMap dmap(0, 0);
GridMapping grid_mapping;

// Parameters
float resolution = 0.05;
float max_range = 10;
float expansion_range = 1;
Canvas canvas;

// Publisher
ros::Publisher gridmap_pub;

// Helper function to convert Grid_ data to OccupancyGrid
void convertToOccupancyGrid(const Grid_<float>& grid, nav_msgs::OccupancyGrid& occupancy_grid) {
  occupancy_grid.info.resolution = resolution;
  occupancy_grid.info.width = grid.cols;
  occupancy_grid.info.height = grid.rows;
  occupancy_grid.info.origin.position.x = -grid.cols * resolution / 2;
  occupancy_grid.info.origin.position.y = -grid.rows * resolution / 2;
  occupancy_grid.info.origin.position.z = 0;
  occupancy_grid.info.origin.orientation.w = 1.0;

  occupancy_grid.data.resize(grid.cols * grid.rows);
  for (int y = 0; y < grid.rows; ++y) {
    for (int x = 0; x < grid.cols; ++x) {
      int idx = y * grid.cols + x;
      float value = grid(y, x);
      if (value < 0) {
        occupancy_grid.data[idx] = -1; // Unknown
      } else if (value > 1) {
        occupancy_grid.data[idx] = 100; // Occupied
      } else {
        occupancy_grid.data[idx] = static_cast<int8_t>(value * 100); // Free
      }
    }
  }
}

void laserCallback(const sensor_msgs::LaserScan& scan) {
  std::vector<Vector2i> endpoints;
  float angle = scan.angle_min;

  for (size_t i = 0; i < scan.ranges.size(); ++i) {
    float r = scan.ranges[i];
    if (r < scan.range_min || r > scan.range_max)
      continue;

    float alpha = scan.angle_min + i * scan.angle_increment;
    Eigen::Vector2i endpoint = grid_mapping.world2grid(Vector2f(r * cos(alpha), r * sin(alpha))).cast<int>();
    endpoints.push_back(endpoint);
  }

  dmap.clear();
  int dmax2 = pow(expansion_range / resolution, 2);

  int ops = dmap.compute(endpoints, dmax2);
  cerr << ops;

  Grid_<float> distances;
  dmap.copyTo(distances);

  // Draw the grid on the canvas
  distances.draw(canvas, true);
  showCanvas(canvas, 1);

  // Convert the grid to OccupancyGrid and publish it
  nav_msgs::OccupancyGrid occupancy_grid;
  convertToOccupancyGrid(distances, occupancy_grid);
  gridmap_pub.publish(occupancy_grid);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "gridmap_publisher");
  ros::NodeHandle n;

  string topic_name = argv[1];

  // Publisher for the grid map
  gridmap_pub = n.advertise<nav_msgs::OccupancyGrid>("/gridmap", 1);

  // Compute the grid size
  int grid_size = 2 * (max_range + expansion_range) / resolution;
  dmap.resize(grid_size, grid_size);
  grid_mapping.reset(Vector2f(-grid_size * resolution / 2, grid_size * resolution / 2), resolution);

  cerr << "grid_size: " << grid_size << endl;
  cerr << "world center: " << grid_mapping.world2grid(Vector2f(0, 0)).transpose() << endl;

  ros::Subscriber sub = n.subscribe<const sensor_msgs::LaserScan&>(topic_name, 10, laserCallback);

  ros::spin();
}