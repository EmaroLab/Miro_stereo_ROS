#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>
#include <pluginlib/class_loader.h>

ros::Time image_time; 
ros::Time last_time;
 double x = 0.0;
  double y = 0.0;
  double th = 1.57-1.57/4;
 double th_initial=1.57-1.57/4;
 
class SubscribeAndPublish_odom
{
public:
  SubscribeAndPublish_odom()
  {
    //Topic you want to publish
    pubh_odom = nh_odom.advertise< nav_msgs::Odometry>("/stereo/odom", 1);

    //Topic you want to subscribe
    subh_odom = nh_odom.subscribe("/odom/miro", 1, &SubscribeAndPublish_odom::callback, this);
  }

  void callback(const  nav_msgs::Odometry& input)
  {
    nav_msgs::Odometry output;
	output=input;
	

 
  double vx = input.twist.twist.linear.x/230;
  //double vx = 0.0;
  double vy = 0.0;
  double vth = input.twist.twist.angular.z;

  ros::Time current_time;
  current_time = ros::Time::now();
        //std::cout<<vth<<"vth\n"; 
double absvth = vth >0?vth:-vth;
           
  if ( absvth > 0.0) {  
        //std::cout<<vth<<"vth inside if\n";      
        vx=0;
    }
    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    
    
    double delta_th = vth * dt;
    double ph = 3.14;
    double roll = 1.57;
    

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat_odom = tf::createQuaternionMsgFromRollPitchYaw(roll,ph,th_initial);
    geometry_msgs::Quaternion odom_quat_tf = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;

    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "map";
    odom_trans.child_frame_id = "miro_robot__miro_body__body";
    
//miro_robot__miro_head__eyelid_lh 
    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = odom_quat_tf;
    //then, we'll publish the transform over tf for point cloud.

    geometry_msgs::TransformStamped odom_trans_stereo;
    odom_trans_stereo.header.stamp = odom_trans.header.stamp;
    odom_trans_stereo.header.frame_id = "miro_robot__miro_head__eyelid_lh";
    odom_trans_stereo.child_frame_id = "stereo";
    
 
    odom_trans_stereo.transform.translation.x = 0;
    odom_trans_stereo.transform.translation.y = 0;
    odom_trans_stereo.transform.translation.z = 0;
    odom_trans_stereo.transform.rotation = odom_quat_odom;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
    odom_broadcaster.sendTransform(odom_trans_stereo);

    //next, we'll publish the odometry message over ROS
    output.header.stamp = current_time;
    output.header.frame_id = "map";

    //set the position
    output.pose.pose.position.x = 0;
    output.pose.pose.position.y = 0;
    output.pose.pose.position.z = 0.25;
    output.pose.pose.orientation = odom_quat_odom;

    //set the velocity
    output.child_frame_id = "stereo";

    last_time = current_time;
    pubh_odom.publish(output);
  }

private:
  ros::NodeHandle nh_odom; 
  ros::Publisher pubh_odom;
  ros::Subscriber subh_odom;
  tf::TransformBroadcaster odom_broadcaster;

};//End of class SubscribeAndPublish


class SubscribeAndPublish_r
{
public:
  SubscribeAndPublish_r()
  {
    //Topic you want to publish
    pubh_r = nh_r.advertise<sensor_msgs::CameraInfo>("/stereo/right/camera_info/", 1);

    //Topic you want to subscribe
    subh_r = nh_r.subscribe("/yaml/right/camera_info/", 1, &SubscribeAndPublish_r::callback, this);
  }

  void callback(const sensor_msgs::CameraInfo& input)
  {
    sensor_msgs::CameraInfo output;
	output=input;
	//output.roi.x_offset=0;
	//output.roi.y_offset=0;
	//output.roi.height=240;
	//output.roi.width=140;
	//output.height=240;
	//output.width=140;
	output.header.stamp = image_time;
        output.header.frame_id = "stereo";
    //.... do something with the input and generate the output...
    pubh_r.publish(output);
  }

private:
  ros::NodeHandle nh_r; 
  ros::Publisher pubh_r;
  ros::Subscriber subh_r;

};//End of class SubscribeAndPublish

class SubscribeAndPublish_IMG_r
{
public:
  SubscribeAndPublish_IMG_r()
  {
    //Topic you want to publish
    pub_r = n_r.advertise<sensor_msgs::Image>("/stereo/right/image_raw", 1);

    //Topic you want to subscribe
    sub_r = n_r.subscribe("/miro_scaledimage/right/image_raw", 1, &SubscribeAndPublish_IMG_r::callback, this);
  }

  void callback(const sensor_msgs::Image& input)
  {
    sensor_msgs::Image output;
	output=input;
	image_time=ros::Time::now();
        output.header.stamp = image_time;
        output.header.frame_id = "stereo";
	//output.header.stamp = ros::Time::now();
    //.... do something with the input and generate the output...
    pub_r.publish(output);
  }

private:
  ros::NodeHandle n_r; 
  ros::Publisher pub_r;
  ros::Subscriber sub_r;

};//End of class SubscribeAndPublish


class SubscribeAndPublish_l
{
public:
  SubscribeAndPublish_l()
  {
    //Topic you want to publish
    pubh_l = nh_l.advertise<sensor_msgs::CameraInfo>("/stereo/left/camera_info/", 1);

    //Topic you want to subscribe
    subh_l = nh_l.subscribe("/yaml/left/camera_info/", 1, &SubscribeAndPublish_l::callback, this);
  }

  void callback(const sensor_msgs::CameraInfo& input)
  {
    sensor_msgs::CameraInfo output;
	output=input;
	//output.roi.x_offset=60;
	//output.roi.y_offset=0;
	//output.roi.height=240;
	//output.roi.width=40;
	//output.height=240;
	//output.width=140;
	output.header.stamp = image_time;
        output.header.frame_id = "stereo";
    //.... do something with the input and generate the output...
    pubh_l.publish(output);
  }

private:
  ros::NodeHandle nh_l; 
  ros::Publisher pubh_l;
  ros::Subscriber subh_l;

};//End of class SubscribeAndPublish

class SubscribeAndPublish_IMG_l
{
public:
  SubscribeAndPublish_IMG_l()
  {
    //Topic you want to publish
    pub_l = n_l.advertise<sensor_msgs::Image>("/stereo/left/image_raw", 1);

    //Topic you want to subscribe
    sub_l = n_l.subscribe("/miro_scaledimage/left/image_raw", 1, &SubscribeAndPublish_IMG_l::callback, this);
  }

  void callback(const sensor_msgs::Image& input)
  {
    sensor_msgs::Image output;
	output=input;
	output.header.stamp = image_time;
        output.header.frame_id = "stereo";
	//output.header.stamp = ros::Time::now();
    //.... do something with the input and generate the output...
    pub_l.publish(output);
  }

private:
  ros::NodeHandle n_l; 
  ros::Publisher pub_l;
  ros::Subscriber sub_l;

};//End of class SubscribeAndPublish


int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subpub");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish_r SAPObject_r;
	SubscribeAndPublish_IMG_r SAPObject_IMG_r;
	SubscribeAndPublish_l SAPObject_l;
	SubscribeAndPublish_IMG_l SAPObject_IMG_l;
	SubscribeAndPublish_odom SAPObject_odom;
  ros::spin();

  return 0;
}

