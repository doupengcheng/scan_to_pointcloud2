#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <iostream>
 
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
class SubscribeAndPublish
{
private:
	ros::NodeHandle n;
	ros::Publisher pub;
  sensor_msgs::PointCloud2 laser_finalMsg;
  pcl::PointCloud<pcl::PointXYZI> front_back_cloud_pcl;
	message_filters::Subscriber<sensor_msgs::PointCloud2>* sub_front;
	message_filters::Subscriber<sensor_msgs::PointCloud2>* sub_back;
	
 
public:
    SubscribeAndPublish(){
		pub = n.advertise<sensor_msgs::PointCloud2>("/Addlaserpoint", 10); //定义两个点云数据融合后发布
		sub_front = new message_filters::Subscriber<sensor_msgs::PointCloud2>(n,  "/frontlaserPointCloud", 2000); //接收前雷达转换后的点云数据
		sub_back = new message_filters::Subscriber<sensor_msgs::PointCloud2>(n, "/backlaserPointCloud", 2000);  //接收后雷达转换后的点云数据
		
		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> syncPolicy;
		message_filters::Synchronizer<syncPolicy> sync(syncPolicy(1000), *sub_front, *sub_back);
		sync.registerCallback(boost::bind(&SubscribeAndPublish::callback, this,  _1, _2));
 
		ros::spin();
	}
    //回调函数
    void callback(const sensor_msgs::PointCloud2::ConstPtr& frontlaserPointCloud, const sensor_msgs::PointCloud2::ConstPtr& backlaserPointCloud){
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_frontPointCloud(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*frontlaserPointCloud, *pcl_frontPointCloud);  //将接收到的前雷达点云数据转换成pcl格式的点云数据
 
      pcl::PointCloud<pcl::PointXYZI>::Ptr front_calibration_cloud(new pcl::PointCloud<pcl::PointXYZI>()); //定义一个新的前雷达的点云数据
      Eigen::Matrix4f g_front_calibration_matrix = Eigen::Matrix4f::Identity();   //变换矩阵
				g_front_calibration_matrix(0, 0) = 1; 
				g_front_calibration_matrix(0, 1) = 0;
				g_front_calibration_matrix(0, 2) = 0;
				g_front_calibration_matrix(0, 3) = 0;
				g_front_calibration_matrix(1, 0) = 0; 
				g_front_calibration_matrix(1, 1) = 1;
				g_front_calibration_matrix(1, 2) = 0;
				g_front_calibration_matrix(1, 3) = 0;
				g_front_calibration_matrix(2, 0) = 0;
				g_front_calibration_matrix(2, 1) = 0;
				g_front_calibration_matrix(2, 2) = 1;
				g_front_calibration_matrix(2, 3) = 0;
                g_front_calibration_matrix(3, 0) = 0;
				g_front_calibration_matrix(3, 1) = 0;
				g_front_calibration_matrix(3, 2) = 0;
				g_front_calibration_matrix(3, 3) = 1;
      
 
      pcl::PointCloud<pcl::PointXYZI>::Ptr back_calibration_cloud(new pcl::PointCloud<pcl::PointXYZI>());  //定义一个新的后雷达的点云数据
      Eigen::Matrix4f g_back_calibration_matrix = Eigen::Matrix4f::Identity();    //后雷达的变换矩阵
	  float theta = M_PI; 
	  Eigen::Matrix4f transform_pitch;//俯仰
      Eigen::Matrix4f transform_roll;//翻转
      Eigen::Matrix4f transform_yaw;//偏航
	  			transform_pitch << 1, 0, 0, 0, \
                        0, cos(0), -sin(0), 0, \
                        0, sin (0), cos (0), 0, \
                        0, 0, 0, 1;
				transform_roll << cos (0), 0, sin(0), 0, \
                     0, 1, 0, 0, \
                     -sin (0), 0, cos (0), 0, \
                     0, 0, 0, 1;
				transform_yaw << cos (theta), -sin(theta), 0, 0, \
                     sin (theta), cos (theta), 0, 0, \
                     0, 0, 1, 0, \
                     0, 0, 0, 1;
				g_back_calibration_matrix = transform_yaw*transform_roll*transform_pitch;    //后雷达完整的变换矩阵
				g_back_calibration_matrix (0,3) = 0.15;  //x轴的偏移
   				g_back_calibration_matrix (1,3) = 0;  //y轴的偏移
   				g_back_calibration_matrix (2,3) = 0;  //z轴的偏移
				
      pcl::transformPointCloud(*pcl_frontPointCloud, *front_calibration_cloud, g_front_calibration_matrix);  //将*pcl_frontPointCloud变换为*front_calibration_cloud，变换矩阵为g_front_calibration_matrix。
      for (std::size_t i = 0; i < front_calibration_cloud->size(); ++i)
      {
           front_calibration_cloud->points[i].intensity = 64;
      }      
 
      //back 雷达
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_backPointCloud(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*backlaserPointCloud, *pcl_backPointCloud);
 
      
      pcl::transformPointCloud(*pcl_backPointCloud, *back_calibration_cloud, g_back_calibration_matrix);
      for (std::size_t i = 0; i < back_calibration_cloud->size(); ++i)
      {
           back_calibration_cloud->points[i].intensity = 128;
      }
 
      pcl::PointCloud<pcl::PointXYZI>::Ptr front_back_middle_calibration_cloud(new pcl::PointCloud<pcl::PointXYZI>());  //定义两个点云数据融合后的新点云数据
      *front_back_middle_calibration_cloud = *front_calibration_cloud + *back_calibration_cloud;  //两个点云数据融合
      front_back_cloud_pcl = *front_back_middle_calibration_cloud;
      pcl::toROSMsg(front_back_cloud_pcl, laser_finalMsg); //将pcl格式的点云数据转换成ros可以接收到的点云格式
      pub.publish(laser_finalMsg);   //发布转换后的点云数据。
        }
 
};
 
int main(int argc, char** argv){
	ros::init(argc, argv, "AddpointCloud");
 
	SubscribeAndPublish sap;
  ros::spin();
	return 0;
}