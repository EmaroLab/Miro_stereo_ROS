# Miro_stereo_ROS
>Repository with ROS packages for implementation of Stereo vision on the MIRO robot. It can be used to generate a Point Cloud using the Stereo cameras of MiRo. Futhur it can be used to visualise the point cloud in the Rviz, while simulating the MiRo movement as it moves in the real world using its Odometry messages. 

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

#### Pyhton script for getting Odometry messages
>The platform interface of the MiRo provides the sensor_information of MiRo in form of customized message: **[miro_msgs/platform_sensors]**(miro_msgs/platform_sensors) over topic **platform/sensors**. This also has the Odometry message to be used for the visualsiation of MiRo movement (here done in Rviz). It is considered better to have the messages in [standard form](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html) *(now available in the new Standard interface of MiRo)*. Thus a python script was made to publish **nav_msgs/Odometry** messages. The same python script was made to control the movement of the MiRo robot so that the touch sensors can be used to control the movement. !!!!!!!. This can be done by creating a message of type: **miro_msgs/platform_control** and publishing over topic **platform/control**. Note that it shall be saved and run inside the *mdk/* directory. **(we furthur recommend to put the file inside the same folder as the example python scripts ~/mdk/bin/shared)**

- **miro_get_odom.py** 
  - Takes argument as the robot Id(rob01 if only 1 MiRo connected)
  - Subscribes to the topic: "/miro/rob01/platform/sensors".
  - Extracts Odometry info.
  - Publishes over the topic: "/odom/miro".
  - Extracts the touch sensors information and create the corresponding platform_control message.
  - Publishes the platform_control message over topic "miro/rob01/platform/control".

### Using the MiRo Stereo Adaptor
>This is a standalone ROS package that was created exclusively for MiRo stereo vision purpose.It has following executables and corresponding nodes:
- **scaleimage_left.cpp**
- **scaleimage_right.cpp**
- **subpub.cpp**
- **camera_info_publisher_left.py**
- **camera_info_publisher_right.py**

> The MiRo-MDK publishes 2 images form its left and right eye cameras, over topics “miro/rob01/platform/caml” and “miro/rob01/platform/camr”. But it does not publishes the much required [Camera_info](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html) messages for these. So we had to do the calibration to find the Camera parameters (saved as .yaml file). Before doing this, the images need to be resized/rescaled before calibration, since for stereo algorithm analysis we need only the overlap part of the 2 images(30 degrees overlap for MiRo). We have used Open_cv(CV_bridge for ROS) tools for the resizing/rescaling.

>The camera_info message components involve intrinsic and extrinsic parts. The intrinsic parts are related to single camera and can be found from [monocular calibration](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration). However the extrinsic parts are related to stereo camera exclusively, and for that we need proper stereo_calibration to be done and shall verify the rectified image. We used the rospackage: [Camera_calibration](http://wiki.ros.org/camera_calibration) and followed the corresponding [tutorial](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration) for stereo calibration. **It shall be kept in mind that we need to get the camera_info message parameters for the Scaled_MiRo images since that would used for furthur processing.** As the outpout we will get yaml files for left and right camera_info messages. This can be used to publish the camera_info message over a topic.

>Furthur we need to synchronize all data. All the MiRo Messages are produced without a [header](http://docs.ros.org/lunar/api/std_msgs/html/msg/Header.html):frame_id, which is very important for stereo processing packages and visualisation packages/softwares. Similarily the time_stamps of images are not necessarily same. So, before Stereo-processing, we take all incoming messages(Scaled images, Camera_info and Odomerty messages) and republish them by giving the frame_id **stereo** to images and camera_info messages and same time stamp to all of them *(we copy the time stamp of the odometry message to distribute them over all others, thus giving priority to the Odometry. Same can also be done by giving the priority to one of the left/right image. Otherwise we can generate new time stamp corresponding to current time.)*

> For visualisation we also need to broadcast 
>Thus executables/nodes: 
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
 > Run by : 
