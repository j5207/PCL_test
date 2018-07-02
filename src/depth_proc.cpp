#include <ros/ros.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/registration/icp.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl_ros/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>




typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef sensor_msgs::PointCloud2 RosCloud;

class TestKinect
{
  private:
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    ros::Subscriber sub_cloud;
    ros::Publisher pub_cloud;
    image_transport::Subscriber sub_image;
    image_transport::Publisher pub_image;
    image_transport::Subscriber sub_depth;
    image_transport::Publisher pub_depth;
  public:
    TestKinect() : it(nh)
    {
      sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1, &TestKinect::CloudCallback, this);
      sub_image = it.subscribe("/camera/rgb/image_rect_color", 1, &TestKinect::ImageCallback, this);
      pub_image = it.advertise("/TestKinect/image", 1);
      pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/TestKinect/cloud", 1);
      sub_depth = it.subscribe("/camera/depth/image_rect", 1, &TestKinect::DepthCallback, this);
      pub_depth = it.advertise("/TestKinect/depth", 1);

    }
    void DepthCallback(const sensor_msgs::ImageConstPtr &msg)
    {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "32FC1");
      // ROS_INFO("the rows of depth is %i", cv_ptr->image.rows);
      // ROS_INFO("the cols of depth is %i", cv_ptr->image.cols);
    }

   void CloudCallback(const RosCloud::ConstPtr& ros_cloud_in)
    {
      pcl::PCLPointCloud2::Ptr pc_in(new pcl::PCLPointCloud2());
      pcl::PCLPointCloud2::Ptr pc_out(new pcl::PCLPointCloud2());
      pcl::PCLPointCloud2::Ptr final_out(new pcl::PCLPointCloud2());

      pcl::PointCloud<pcl::PointXYZ>::Ptr temp_pc(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr temp_out(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr temp_haha(new pcl::PointCloud<pcl::PointXYZ>);

      pcl::PointCloud <pcl::PointXYZRGB>::Ptr rgb_pc (new pcl::PointCloud <pcl::PointXYZRGB>);
      
      pcl::PointCloud<pcl::PointXYZ>::Ptr filter_pc(new pcl::PointCloud<pcl::PointXYZ>);
      pcl_conversions::toPCL(*ros_cloud_in, *pc_in);
      pcl::fromPCLPointCloud2(*pc_in, *filter_pc);

      // Create the VoxelGrid filtering object
      pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
      sor.setInputCloud(pc_in);
      sor.setLeafSize(0.01f, 0.01f, 0.01f);
      sor.filter(*pc_out);

      std::cout << "heng" << std::endl;
      // Convert pointcloud2 to template pcl
      pcl::fromPCLPointCloud2(*pc_out, *rgb_pc);

      std::cout << "kuku" << std::endl;


      pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);


      pcl::IndicesPtr indices (new std::vector <int>);
      pcl::PassThrough<pcl::PointXYZRGB> pass;
      pass.setInputCloud (rgb_pc);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0.0, 1.0);
      pass.filter (*indices);

      pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
      reg.setInputCloud (rgb_pc);
      reg.setIndices (indices);
      reg.setSearchMethod (tree);
      reg.setDistanceThreshold (10);
      reg.setPointColorThreshold (6);
      reg.setRegionColorThreshold (5);
      reg.setMinClusterSize (600);

      std::vector <pcl::PointIndices> clusters;
      reg.extract (clusters);

      pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();

      // Create StatisticalOutlierRemoval
      // pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
      // sor.setInputCloud(pc_in);
      // sor.setMeanK(50);
      // sor.setStddevMulThresh(1.0);
      // sor.filter(*pc_out);
      // Convert back to sensor msg
      pcl::toPCLPointCloud2(*colored_cloud, *final_out);
      sensor_msgs::PointCloud2::Ptr msg_out(new sensor_msgs::PointCloud2());
      pcl_conversions::fromPCL(*final_out, *msg_out);
      pub_cloud.publish(msg_out);


    }


    void ImageCallback(const sensor_msgs::ImageConstPtr & msg)
    {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      cv::Mat hsv_image; 
      cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);
      cv::inRange(hsv_image, cv::Scalar(128, 38, 0), cv::Scalar(139, 255, 255), hsv_image);
      // if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      //     cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));
      sensor_msgs::ImagePtr out_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", hsv_image).toImageMsg();
      pub_image.publish(out_msg);

      // ROS_INFO("the rows of color is %i", cv_ptr->image.rows);
      // ROS_INFO("the cols of color is %i", cv_ptr->image.cols);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_proc");
    TestKinect test;
    ros::spin();

}