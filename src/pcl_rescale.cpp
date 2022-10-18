#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/approximate_voxel_grid.h"
#include "pcl/filters/conditional_removal.h"
#include "pcl/filters/crop_hull.h"
#include "pcl/common/io.h"
#include "pcl/Vertices.h"
#include "pcl/common/pca.h"
#include "pcl/common/transforms.h"
#include "pcl/common/common.h"

//#define POINTS_TOPIC "/camera/aligned_depth_to_color/color/points"
#define POINTS_TOPIC "/velodynes/left/points"
#define CENTER_POINTS_TOPIC "/velodybes/center/points"
#define ROBOT_HEIGHT 2.8
#define POINT_TYPE pcl::PointXYZ

using namespace std::chrono_literals;

void rescaleClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr& goldenCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& sampleCloud, bool debugOverlay = false, bool primaryAxisOnly = true) {
    //analyze golden cloud
    pcl::PCA<pcl::PointXYZ> pcaGolden;
    pcaGolden.setInputCloud(goldenCloud);
    Eigen::Matrix3f goldenEVs_Dir = pcaGolden.getEigenVectors();
    Eigen::Vector4f goldenMidPt = pcaGolden.getMean();
    Eigen::Matrix4f goldenTransform = Eigen::Matrix4f::Identity();
    goldenTransform.block<3, 3>(0, 0) = goldenEVs_Dir;
    goldenTransform.block<4, 1>(0, 3) = goldenMidPt;
    pcl::PointCloud<pcl::PointXYZ>::Ptr orientedGolden(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*goldenCloud, *orientedGolden, goldenTransform.inverse());
    pcl::PointXYZ goldenMin, goldenMax;
    pcl::getMinMax3D(*orientedGolden, goldenMin, goldenMax);

    //analyze sample cloud
    pcl::PCA<pcl::PointXYZ> pcaSample;
    pcaSample.setInputCloud(sampleCloud);
    Eigen::Matrix3f sampleEVs_Dir = pcaSample.getEigenVectors();
    Eigen::Vector4f sampleMidPt = pcaSample.getMean();
    Eigen::Matrix4f sampleTransform = Eigen::Matrix4f::Identity();
    sampleTransform.block<3, 3>(0, 0) = sampleEVs_Dir;
    sampleTransform.block<4, 1>(0, 3) = sampleMidPt;
    pcl::PointCloud<pcl::PointXYZ>::Ptr orientedSample(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*sampleCloud, *orientedSample, sampleTransform.inverse());
    pcl::PointXYZ sampleMin, sampleMax;
    pcl::getMinMax3D(*orientedSample, sampleMin, sampleMax);

    //apply scaling to oriented sample cloud 
    double xScale = (sampleMax.x - sampleMin.x) / (goldenMax.x - goldenMin.x);
    double yScale = (sampleMax.y - sampleMin.y) / (goldenMax.y - goldenMin.y);
    double zScale = (sampleMax.z - sampleMin.z) / (goldenMax.z - goldenMin.z);

    if (primaryAxisOnly) { std::cout << "scale: " << xScale << std::endl; }
    else { std::cout << "xScale: " << xScale << "yScale: " << yScale << "zScale: " << zScale << std::endl; }


    for (long unsigned int i = 0; i < orientedSample->points.size(); i++)
    {
        if (primaryAxisOnly)
        {
            orientedSample->points[i].x = orientedSample->points[i].x / xScale;
            orientedSample->points[i].y = orientedSample->points[i].y / xScale;
            orientedSample->points[i].z = orientedSample->points[i].z / xScale;
        }
        else
        {
            orientedSample->points[i].x = orientedSample->points[i].x / xScale;
            orientedSample->points[i].y = orientedSample->points[i].y / yScale;
            orientedSample->points[i].z = orientedSample->points[i].z / zScale;
        }
    }
    //depending on your next step, it may be reasonable to leave this cloud at its eigen orientation, but this transformation will allow this function to scale in place.

    if (debugOverlay)
    {
        goldenCloud = orientedGolden;
        sampleCloud = orientedSample;
    }
    else
    {
        pcl::transformPointCloud(*orientedSample, *sampleCloud, sampleTransform);
    }
}


class PclFilter : public rclcpp::Node
{
  public:
    PclFilter()
    : Node("pcl_rescaler"), count_(0)
    {
      pcl_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        POINTS_TOPIC, 10, std::bind(&PclFilter::pcl_callback, this, std::placeholders::_1));

      center_pcl_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        CENTER_POINTS_TOPIC, 10, std::bind(&PclFilter::center_pcl_callback, this, std::placeholders::_1));

      rescaled_points_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("velodynes/center/points/rescaled", 10);


      grid.setLeafSize(1.15, 1.15, 1.15);
    }

  private:
    void pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      // This will convert the message into a pcl::PointCloud
      pcl::fromROSMsg(*msg, *cloud);
      rescaleClouds(cloud, center_cloud);
      grid.filter(*cloud);
    }

    void center_pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      pcl::fromROSMsg(*msg, *center_cloud);
      sensor_msgs::msg::PointCloud2 rescaled_points_msg;
      pcl::toROSMsg(*rescaled_cloud, rescaled_points_msg);
      rescaled_points_publisher->publish(rescaled_points_msg);
    }

    void timer_callback() { }

    pcl::PointCloud<POINT_TYPE>::Ptr cloud = boost::make_shared<pcl::PointCloud<POINT_TYPE>>();
    pcl::PointCloud<POINT_TYPE>::Ptr center_cloud = boost::make_shared<pcl::PointCloud<POINT_TYPE>>();
    pcl::PointCloud<POINT_TYPE>::Ptr rescaled_cloud = boost::make_shared<pcl::PointCloud<POINT_TYPE>>();
    pcl::ApproximateVoxelGrid<POINT_TYPE> grid = pcl::ApproximateVoxelGrid<POINT_TYPE>();

    pcl::CropHull<POINT_TYPE> obstacle_hull_filter;

    pcl::ConditionAnd<POINT_TYPE>::Ptr z_obstacle_cond;
    pcl::ConditionAnd<POINT_TYPE>::Ptr z_traversible_cond;
    pcl::ConditionalRemoval<POINT_TYPE> spatial_obstacle_filter = pcl::ConditionalRemoval<POINT_TYPE>();
    pcl::ConditionalRemoval<POINT_TYPE> spatial_traversible_filter = pcl::ConditionalRemoval<POINT_TYPE>();


    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr center_pcl_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rescaled_points_publisher;


    rclcpp::TimerBase::SharedPtr timer_;
  /*    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;*/
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PclFilter>());
  rclcpp::shutdown();
  //N::helloworld();
  return 0;
}