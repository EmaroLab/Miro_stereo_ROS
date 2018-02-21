 /**
 * \file pcl_filter_miro.cpp
 * \brief A node to filter the noise of a point cloud.
 * \author John Thomas, Parag Khanna
 * \version 0.1
 * \date 11 Feb 2017
 * 
 * \param[in] ...
 * 
 * Subscribes to: /stereo/points2 <BR>
 * 
 * Publishes to: /pcl_filtered_miro<BR>
 *
 *  \b It subscribes to miro point cloud topic. points_arg takes the pathname of the topic.
 *  For example : points_arg = "/stereo/points2". if stereo_image_proc is used.
 */

#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <iostream>

char* points_arg;

class cloudHandler
{
public:
    cloudHandler()
    {
       
        pcl_sub = nh.subscribe(points_arg, 10, &cloudHandler::cloudCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_filtered_miro", 1);
//"/stereo/points2"
    }

    void cloudCB(const sensor_msgs::PointCloud2& input)
    {
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::PointCloud<pcl::PointXYZRGB> cloud_filtered;
        sensor_msgs::PointCloud2 output;
       
        pcl::fromROSMsg(input, cloud);
        
        //pcl::removeNaNFromPointCloud(cloud);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> statFilter;
        statFilter.setInputCloud(cloud.makeShared());
        statFilter.setMeanK(10);
        statFilter.setStddevMulThresh(0.2);
        statFilter.filter(cloud_filtered);

        pcl::toROSMsg(cloud_filtered, output);
        pcl_pub.publish(output);
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
};

main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_filter_miro");
    points_arg = argv[1];
    // std::cout << argv[1] << "\n";
    cloudHandler handler;

    ros::spin();

    return 0;
}

