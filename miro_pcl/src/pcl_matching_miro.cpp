/**
 * \file pcl_matching_miro.cpp
 * \brief A node to stitch multiple point clouds.
 * \author John Thomas, Parag Khanna
 * \version 0.1
 * \date 7 Feb 2018
 * 
 * \param[in] ...
 * 
 * Subscribes to: pcl_downsampled<BR>
 * 
 * Publishes to: pcl_matched_miro<BR>
 *
 *    It stitches point clouds using basic registration from pcl.
 * 
 */


#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

int count=0;

pcl::PointCloud<pcl::PointXYZRGB> cloud_tmp;

class cloudHandler
{
public:
    cloudHandler()
    {
        pcl_sub = nh.subscribe("pcl_downsampled", 10, &cloudHandler::cloudCB, this);
        pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pcl_matched_miro", 1);
    }

    void cloudCB(const sensor_msgs::PointCloud2 &input)
    {
        
        pcl::PointCloud<pcl::PointXYZRGB> cloud_in;
        pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
        pcl::PointCloud<pcl::PointXYZRGB> cloud_aligned;
        
        sensor_msgs::PointCloud2 output;

        
        pcl::fromROSMsg(input, cloud_in);
        if (count < 5 ) {
           cloud_tmp = cloud_in;
           count = count + 1;
        }
        
        cloud_out=cloud_tmp;
              
           
        pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
        icp.setInputSource(cloud_in.makeShared());
        icp.setInputTarget(cloud_out.makeShared());

        icp.setMaxCorrespondenceDistance(5);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon (1e-12);
        icp.setEuclideanFitnessEpsilon(0.1);

        icp.align(cloud_aligned);
        cloud_tmp = cloud_out + cloud_aligned;

        pcl::toROSMsg(cloud_tmp, output);
        pcl_pub.publish(output);
          
       
    }

protected:
    ros::NodeHandle nh;
    ros::Subscriber pcl_sub;
    ros::Publisher pcl_pub;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_matching_miro");
    
    cloudHandler handler;
   

    ros::spin();

    return 0;
}
