# Miro_stereo_ROS
>Repository with ROS packages for implementation of Stereo vision on the MIRO robot.
The package aims to generate a Point Cloud using the Stereo cameras of MiRo. Futhur it can be used to visualise the point cloud in the Rviz, while simulating the MiRo movement as it moves in the real world using its Odometry messages. 

### Start working with MiRo
>To work with MiRo, we need to prepare our workstation for MiRo, via installing the MIRO Developer Kit(MDK) and configuring the installation (ROS and/or Gazebo) for use with MiRo.
- The simple steps to be followed for above are listed on the [MiRo website](https://consequential.bitbucket.io/Developer_Preparation_Prepare_workstation.html).
- Finally by using MiRo-APP, we can connect to MiRo via BLUETOOTH from a Smartphone, and then can make MiRo connect to the desired wifi network on which our workstation would be connected. Furthur we need to Commission the MiRo for use, following these [steps](https://consequential.bitbucket.io/Developer_Preparation_Commission_MIRO.html). After this we also need to set the *MODE* to *Normal* in MiRo-APP and toggle the *BRIDGE* on. 
- If everything goes as desired, once roscore is run in the ROS Environment in our workstation we finally have a Rosnode running which corresponds to various interfaces of MiRo:Standard(new), Platform, Core and Bridge. We will be able to see various *Topics* related to MiRo in `rostopic list`
- We can verify this by running the example python script in the mdk directory created while preparing the workstation.
  - `cd ~/mdk/bin/shared/`
  - `./miro_ros_client_gui.py robot=rob01`

>We would be using the [platform interface](https://consequential.bitbucket.io/Technical_Interfaces_Platform_Interface.html) of the MiRo in this approach.

#### Pyhton script for getting Odometry messages
>The platform interface of the MiRo provides the sensor_information of MiRo in form of customized message: [miro_msgs/platform_sensors](miro_msgs/platform_sensors). This also has the Odometry message to be used for the visualsiation of MiRo movement (here done in Rviz). 

### Using the MiRo Stereo Adaptor
>This is a standalone ROS package that was created exclusively for MiRo stereo vision purpose.It has following executables and corresponding nodes:
- **scaleimage_left.cpp**
- **scaleimage_right.cpp**
- **subpub.cpp**
- **camera_info_publisher_left.py**
- **camera_info_publisher_right.py**

> The MiRo-MDK publishes 2 images form its left and right eye cameras, over topics “miro/rob01/platform/caml” and “miro/rob01/platform/camr”. But it does not publishes the much required [Camera_info](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html) messages for these. So we had to do the calibration to find the Camera parameters (saved as .yaml file). Before doing this, the images need to be resized/rescaled before calibration, since for stereo algorithm analysis we need only the overlap part of the 2 images(30 degrees overlap for MiRo). We have used Open_cv(CV_bridge for ROS) tools for the resizing/rescaling.

>The camera_info message components involve intrinsic and extrinsic parts. The intrinsic parts are related to single camera and can be found from [monocular calibration](). However the extrinsic parts are related to stereo camera exclusively, and for that we need proper stereo_calibration to be done and shall verify the rectified image. We used the rospackage: [Camera_calibration](http://wiki.ros.org/camera_calibration) and followed the corresponding [tutorial](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration) for stereo calibration. **It shall be kept in mind that we need to get the camera_info message parameters for the Scaled_MiRo images since that would used for furthur processing.** As the outpout we will get yaml files for left and right camera_info messages. This can be used to publish the camera_info message over a topic.

>Furthur we need to synchronize all data. All the MiRo Messages are produced without a [header](http://docs.ros.org/lunar/api/std_msgs/html/msg/Header.html):frame_id, which is very important for stereo processing packages and visualisation packages/softwares. Similarily the time_stamps of images can be different So, before Stereo-processing, we take all incoming messages(Scaled images, Camera_info and Odomerty message) 

>Thus executables/nodes: 
 - **scaleimage_left and scaleimage_right**
   - Subcribes to “miro/rob01/platform/caml” and “miro/rob01/platform/camr”
   - resize/rescale the images 
   - Publishes the output over "/miro_scaledimage/left/image_raw" and "/miro_scaledimage/right/image_raw"
 - **camera_info_publisher_left and camera_info_publisher_right**
   - Reads from .yaml file for various components of camera_info msg 
   - Publishes the camera_info message over topic ""/""
 - **subpub**
   - 
