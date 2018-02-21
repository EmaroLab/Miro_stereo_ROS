#!/usr/bin/env python

"""

 * file camera_info_publisher_left.py
 * brief A generate and publish the camera_info Messages from a .yaml file. 
 * authors Parag Khanna, John Thomas
 * version 0.1
 * \date  11 February 2018
 * 
 * \param[in] ...
 * 
 * Subscribes to: ..<BR>
 * 
 * Publishes to: /yaml/left/camera_info<BR>
  
 This node reads in a camera calibration ouput .yaml file (as created by the cameracalibrator.py in the camera_calibration pkg),
 and converts it to a sensor_msgs/CameraInfo message, and publish it on a topic.

"""


import rospy
import sys
import yaml
from sensor_msgs.msg import CameraInfo

def yaml_to_CameraInfo(yaml_fname):
    """
    Parse a yaml file containing camera calibration data (as produced by 
    rosrun camera_calibration cameracalibrator.py) into a 
    sensor_msgs/CameraInfo msg.
    
    Parameters
    ----------
    yaml_fname : str
        Path to yaml file containing camera calibration data

    Returns
    -------
    camera_info_msg : sensor_msgs.msg.CameraInfo
        A sensor_msgs.msg.CameraInfo message containing the camera calibration
        data
    """
    # Load data from file
    with open(yaml_fname, "r") as file_handle:
        calib_data = yaml.load(file_handle)
    # Parse
    camera_info_msg = CameraInfo()
    camera_info_msg.width = calib_data["image_width"]
    camera_info_msg.height = calib_data["image_height"]
    camera_info_msg.K = calib_data["camera_matrix"]["data"]
    camera_info_msg.D = calib_data["distortion_coefficients"]["data"]
    camera_info_msg.R = calib_data["rectification_matrix"]["data"]
    camera_info_msg.P = calib_data["projection_matrix"]["data"]
    camera_info_msg.distortion_model = calib_data["distortion_model"]
    return camera_info_msg

if __name__ == "__main__":
    args=rospy.myargv(argv=sys.argv)
    #remove the ros-induced arguments in case of ros-launching
    # print "The arguments are: " , str(args)
    # Get fname from command line (cmd line input required)
    filename=args[1]
    # print "The filename is: " , str(filename)
    # Parse yaml file
    camera_info_msg = yaml_to_CameraInfo(filename)

    # Initialize publisher node
    rospy.init_node('camera_info_publisher_left', anonymous=True)
    publisher = rospy.Publisher("/yaml/left/camera_info", CameraInfo, queue_size=10)
    rate = rospy.Rate(4)

    # Run publisher
    while not rospy.is_shutdown():
        publisher.publish(camera_info_msg)
        rate.sleep()

