#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from PIL import Image
import numpy as np

def publish_map():
    rospy.init_node('map_publisher', anonymous=True)
    pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
    
    # Load the map image
    map_image = Image.open("cappero_laser_odom_diag_2020-05-06-16-26-03.png")
    map_data = np.array(map_image.convert('L'))  # Convert to grayscale
    map_data = (100 - map_data / 2.55).astype(np.int8)  # Invert and scale
    
    # Create OccupancyGrid
    map_msg = OccupancyGrid()
    map_msg.info = MapMetaData()
    map_msg.info.resolution = 0.02  # Match .world file
    map_msg.info.width = map_data.shape[1]
    map_msg.info.height = map_data.shape[0]
    map_msg.info.origin.position.x = -53.4565  # Match .world file
    map_msg.info.origin.position.y = -24.67635
    map_msg.info.origin.position.z = 0.0
    map_msg.info.origin.orientation.w = 1.0
    map_msg.data = map_data.flatten().tolist()
    
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(map_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_map()
    except rospy.ROSInterruptException:
        pass
