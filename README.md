# Miro_stereo_ROS
>Repository with ROS packages for implementation of Stereo vision on the MIRO robot. It can be used to generate a Point Cloud using the Stereo cameras of MiRo. Futhur it can be used to visualise the point cloud in the Rviz, while simulating the MiRo movement as it moves in the real world using its Odometry messages. 

## Quick use :

1. All 4 folders: "miro_pcl", "miro_stereo_adaptor", "miro_urdf_launcher" and "stereo_image_proc" are standalone ROS-packages, hence are to be placed in src folder of catkin workspace.
2. Do catkin_make
3. Establish connection with MiRo, verify by :
```
$ cd ~/mdk/bin/shared/
$ ./miro_ros_client_gui.py robot=rob01`
```
4. Run Pyhton script for getting Odometry messages from inside same directory as above:
```
$ cd ~/mdk/bin/shared/
$ ./miro_get_odom.py robot=rob01
```
5. Run Miro_stereo_adaptor by using roslaunch:
```
$ roslaunch miro_stereo_adaptor subpub.launch left_camera_yaml:=scripts/left.yaml right_camera_yaml:=scripts/right.yaml
```
Verify by: 
```
$ rostopic echo /yaml/left/camera_info
$ rostopic echo /stereo/left/camera_info/
$ rosrun image_view image_view image:=/miro_scaledimage/left/image_raw
$ rosrun image_view image_view image:=/stereo/left/image_raw
$ ROS_NAMESPACE=stereo/left rosrun image_proc image_proc
$ rosrun image_view image_view image:=stereo/left/image_rect_color
``` 
6. Run Stereo_image_proc
`$ roslaunch stereo_image_proc miro_stereo_image_proc.launch`
Verify by seeinf the Disparity image that pops up and point cloud in Rviz.
 - Set stereo_parameters in the dynamic rqt_reconfigure by loading the file *miro_stereo_proc_params.yaml* for stereo_image_proc in rqt_reconfigure.

7. Run Miro_urdf_launcher by using roslaunch:
```
$ roslaunch miro_urdf_launcher miro_urdf.launch
```
Visualise the Robot model in Rviz.
8. Run miro_pcl by roslaunch :
```
$ roslaunch miro_pcl pcl4miro.launch topic_points2:="/stereo/points2"
```
Now can see the output filtered/downsampled/original(/stereo/points2) point clouds as well as the Miro visualised in Rviz. The MiRo can be moved around by GUI above or by touching the touch sensors(see details) on its body to move it, and correspondingly the MiRo in Rviz shall move around in the same way (as per recieved Odometry Message). Also the 3 above point clouds too move around as MiRo moves. Also the pcl_matched_miro Point cloud that shall have given stitched larger point cloud as MiRo moves, can be seen not working well, mostly since the Stereo_overlap region is less, thus lesser matching features. Still, it is implemented and published for furthur experiments

## DETAILED DESCRIPTION:

### Start working with MiRo
>To work with MiRo, we need to prepare our workstation for MiRo, via installing the MIRO Developer Kit(MDK) and configuring the installation (ROS and/or Gazebo) for use with MiRo.
- The simple steps to be followed for above are listed on the [MiRo website](https://consequential.bitbucket.io/Developer_Preparation_Prepare_workstation.html).
- Finally by using MiRo-APP, we can connect to MiRo via BLUETOOTH from a Smartphone, and then can make MiRo connect to the desired wifi network on which our workstation would be connected. Furthur we need to Commission the MiRo for use, following these [steps](https://consequential.bitbucket.io/Developer_Preparation_Commission_MIRO.html). After this we also need to set the *MODE* to *Normal* in MiRo-APP and toggle the *BRIDGE* on. 
- If everything goes as desired, once roscore is run in the ROS Environment in our workstation we finally have a Rosnode running which corresponds to various interfaces of MiRo:Standard(new), Platform, Core and Bridge. We will be able to see various *Topics* related to MiRo in `rostopic list`
- We can verify this by running the example python script in the mdk directory created while preparing the workstation.
```
cd ~/mdk/bin/shared/
./miro_ros_client_gui.py robot=rob01`
```
>We would be using the [platform interface](https://consequential.bitbucket.io/Technical_Interfaces_Platform_Interface.html) of the MiRo in this approach.

### Pyhton script for getting Odometry messages
>The platform interface of the MiRo provides the sensor_information of MiRo in form of customized message: **[miro_msgs/platform_sensors]**(miro_msgs/platform_sensors) over topic **platform/sensors**. This also has the Odometry message to be used for the visualsiation of MiRo movement (here done in Rviz). It is considered better to have the messages in [standard form](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html) *(now available in the new Standard interface of MiRo)*. Thus a python script was made to publish **nav_msgs/Odometry** messages. The same python script was made to control the movement of the MiRo robot so that the touch sensors can be used to control the movement. The two head sensors adjacent to the left ear of MiRo  is used for rotating to the left side and the other two sensors near the right for rotating towards the right. The four body sensors are used for translation with the 2 sensors closer to the neck for forward motion and the other two adjacent to the tail for backward movement. This can be done by creating a message of type: **miro_msgs/platform_control** and publishing over topic **platform/control**. Note that it shall be saved and run inside the *mdk/* directory. **(we furthur recommend to put the file inside the same folder as the example python scripts ~/mdk/bin/shared)**

- **miro_get_odom.py** 
  - Takes argument as the robot Id(rob01 if only 1 MiRo connected)
  - Subscribes to the topic: "/miro/rob01/platform/sensors".
  - Extracts Odometry info.
  - Publishes over the topic: "/odom/miro".
  - Extracts the touch sensors information and create the corresponding platform_control message.
  - Publishes the platform_control message over topic "miro/rob01/platform/control".

#### Run by : 
```
cd ~/mdk/bin/shared/
./miro_get_odom.py robot=rob01
```

### miro_urdf_launcher
> A standalone package using URDF file for Miro to visualise it as robot model in Rviz. A corresponding joint state-publisher node has been included as well which correponds to the joints of robot visualised in Rviz, broadcasting various tf like *miro_robot__miro_body__body*,*miro_robot__miro_head__eyelid_lh* etc which all can be seen in Rviz.
 
> An URDF file has not been provided alongwith the Miro-MDK. The visualisation described on MiRo website in Gazebo is via a given SDF file. Thus a conversion was needed. This was done via a Ros_package [pysdf](http://wiki.ros.org/pysdf)

#### Run By:
```
roslaunch miro_urdf_launcher miro_urdf.launch
```
 
**NOTE: The visualised MIRO in Rviz has not been linked to actual robot. The odometry information from robot will be used furthur to get the *miro_robot__miro_body__body* frame moving around as the robot moves**

### Using the MiRo Stereo Adaptor
>This is a standalone ROS package that was created exclusively for MiRo stereo vision purpose.It has following executables and corresponding nodes:
- **scaleimage_left.cpp**
- **scaleimage_right.cpp**
- **subpub.cpp**
- **camera_info_publisher_left.py**
- **camera_info_publisher_right.py**

> The MiRo-MDK publishes 2 images form its left and right eye cameras, over topics “miro/rob01/platform/caml” and “miro/rob01/platform/camr”. But it does not publishes the much required [Camera_info](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html) messages for these. So we had to do the calibration to find the Camera parameters (saved as .yaml file). Before doing this, the images need to be resized/rescaled before calibration, since for stereo algorithm analysis we need only the overlap part of the 2 images(30 degrees overlap for MiRo). We have used Open_cv(CV_bridge for ROS) tools for the resizing/rescaling.

>The camera_info message components involve intrinsic and extrinsic parts. The intrinsic parts are related to single camera and can be found from [monocular calibration](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration). However the extrinsic parts are related to stereo camera exclusively, and for that we need proper stereo_calibration to be done and shall verify the rectified image. We used the rospackage: [Camera_calibration](http://wiki.ros.org/camera_calibration) and followed the corresponding [tutorial](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration) for stereo calibration. **It shall be kept in mind that we need to get the camera_info message parameters for the Scaled_MiRo images since that would used for furthur processing.** As the outpout we will get yaml files for left and right camera_info messages. This can be used to publish the camera_info message over a topic.

> Furthur we need to synchronize all data. All the MiRo Messages are produced without a [header](http://docs.ros.org/lunar/api/std_msgs/html/msg/Header.html):frame_id, which is very important for stereo processing packages and visualisation packages/softwares. Similarily the time_stamps of images are not necessarily same. So, before Stereo-processing, we take all incoming messages(Scaled images, Camera_info and Odomerty messages) and republish them by giving the frame_id **stereo** to images and camera_info messages and same time stamp to all of them *(we copy the time stamp of the odometry message to distribute them over all others, thus giving priority to the Odometry. Same can also be done by giving the priority to one of the left/right image. Otherwise we can generate new time stamp corresponding to current time.)*

> For visualisation in Rviz it is necessary to broadcast the tf between miro Body and fixed map frame, and also between Stereo and a Frame on Miro_robot, with respect to which the Point Cloud will be fixed/visualised.  

> Thus executables/nodes: 
 - **scaleimage_left and scaleimage_right**
   - Subcribes to “miro/rob01/platform/caml” and “miro/rob01/platform/camr”
   - resize/rescale the images 
   - Publishes the output over "/miro_scaledimage/left/image_raw" and "/miro_scaledimage/right/image_raw"
 - **camera_info_publisher_left and camera_info_publisher_right**
   - Takes argument as the path to the .yaml file
   - Reads from .yaml file for various components of camera_info msg 
   - Publishes the camera_info message over topic "/yaml/left/camera_info" / "/yaml/right/camera_info"
 - **subpub**
   - Subscribes to "/miro_scaledimage/left/image_raw" and "/miro_scaledimage/right/image_raw"
   - Add the frame_id and timestamp and re-publishes images over : "/stereo/left/image_raw" and "/stereo/right/image_raw"
   - Subscribes to "/yaml/left/camera_info" and "/yaml/right/camera_info" 
   - Add the frame_id and timestamp and re-publishes camera_info messages over : "/stereo/left/camera_info/" and "/stereo/right/camera_info/" 
   - Subscribes to "/odom/miro", copies the time stamp to be given to other topics.
   - Read the odometry data, set the values for tf accordingly between global frame *map* and *miro_robot__miro_body__body* frame and broadcast it.
   - Also broadcast a static frame between the *stereo* frame and *miro_robot__miro_head__eyelid_lh* (as per the used stereo-processing package)
   - Re-publish the odometry message over : "/stereo/odom" topic.

**NOTE**:
  1. **All republished topics are in "stereo" namespace. This is corresponding to the requirement of stereo_processing packages that need them in same namespace.**
  2. **As we would see ahead, the stereo_processing package used assumes the point cloud produced to be relative to a frame at left eye camera, (X Right, Y Down, Z out).** 
 
#### Run by : 
( a launch file has been provided for whole package, takes arguments as the path for .yaml files for left/right camera_info message files **put inside this package**)

```
roslaunch miro_stereo_adaptor subpub.launch left_camera_yaml:=scripts/left.yaml right_camera_yaml:=scripts/right.yaml
```
> Verify by running following commands: 
```
rostopic echo /yaml/left/camera_info
rostopic echo /stereo/left/camera_info/
rosrun image_view image_view image:=/miro_scaledimage/left/image_raw
rosrun image_view image_view image:=/stereo/left/image_raw
ROS_NAMESPACE=stereo/left rosrun image_proc image_proc
rosrun image_view image_view image:=stereo/left/image_rect_color
```

### [Stereo_image_proc](http://wiki.ros.org/stereo_image_proc) 
> Commonly used open source , standalone ROS package. This requires both camera_info and image topics for both the cameras. It publishes the Rectified images, Disparity Image and the Point Cloud. **The messages shall be synchronized, recommended to not use approximate_sync for best performance**. We used the [tutorial](http://wiki.ros.org/stereo_image_proc/Tutorials/ChoosingGoodStereoParameters) for setting best stereo parameters for the package using [dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure). The stereo_image_proc node and dynamic_reconfigure can be run separately as described in the package summary. **The best parameters ,according to us,  for the Scaled_images from the Miro_robot have been saved in a .yaml file and made available inside this package. This can be direclty loaded in the rqt_reconfigure**. Furthur the output disparity image and rect images can be seen by using **stereo_view node of image_view**. Otherwise a custom launch file was included which launches all above assuming the namespace of the camera/images to be **stereo**. The point cloud can be visualised in Rviz.

#### Run by :
```
ROS_NAMESPACE=stereo rosrun stereo_image_proc stereo_image_proc
rosrun image_view stereo_view stereo:=/stereo image:=image_rect_color
rosrun rqt_reconfigure rqt_reconfigure
```
or 

```
roslaunch stereo_image_proc miro_stereo_image_proc.launch
```

###  miro_pcl

>  Another standalone package that is a collection of nodes implementiong PCL_filtering, PCL_downsampling and PCL_matching from [pcl](http://wiki.ros.org/pcl) Rospackage (Point Cloud Library). These codes have been adapted from the [git](https://github.com/AaronMR/Learning_ROS_for_Robotics_Programming_2nd_edition).  *Due to very narrow stereo_overlap leading to a narrow point cloud, the matching node does not work in desired way. Still it has been publihsed for furthur experiments*. A launch file has been provided which takes in an argument as the point_cloud topic. The nodes work in sequence: 
- pcl_filter_miro.cpp  
  - Takes in the argument as the point cloud topic name
  - Subscribes to it, applying pcl_filtering and publishes the output as "pcl_filtered_miro"
- pcl_downsampling.cpp
  - Subscribes to "pcl_filtered_miro"
  - Applies pcl_downsampling to point cloud and publishes the output as "pcl_downsampled"
- pcl_matching_miro.cpp
  - Subscribes to "pcl_downsampled"
  - It stitches point clouds using basic registration from pcl, publishing the output as "pcl_matched_miro"  

#### Run by:
`roslaunch miro_pcl pcl4miro.launch topic_points2:="/stereo/points2"`

The point clouds can be visualised in Rviz.

Video link : https://drive.google.com/open?id=1rxkzOiZXYNqdtwaGSqynoenknItgD97j
 
