# Miro_stereo_ROS
>Repository with ROS packages for implementation of Stereo vision on the MIRO robot.

### Start working with MiRo
>To work with MiRo, we need to prepare our workstation for MiRo, via installing the MIRO Developer Kit(MDK) and configuring the installation (ROS and/or Gazebo) for use with MiRo.

The simple steps to be followed for above are listed on the [MiRo website](https://consequential.bitbucket.io/Developer_Preparation_Prepare_workstation.html).

Finally by using MiRo-APP, we can connect to MiRo via BLUETOOTH from a Smartphone, and then can make MiRo connect to the desired wifi network on which our workstation would be connected. Furthur we need to Commission the MiRo for use, following these [steps](https://consequential.bitbucket.io/Developer_Preparation_Commission_MIRO.html). After this we also need to set the *MODE* to *Normal* in MiRo-APP and toggle the *BRIDGE* on. 

If everything goes as desired, once roscore is run in the ROS Environment in our workstation we finally have a Rosnode running which corresponds to various interfaces of MiRo:Standard(new), Platform, Core and Bridge. We will be able to see various *Topics* related to MiRo in `rostopic list`

We would be using the [platform interface](https://consequential.bitbucket.io/Technical_Interfaces_Platform_Interface.html) of the MiRo in this approach.

