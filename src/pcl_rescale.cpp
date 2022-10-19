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
// #include "../include/rectangle_filter.hpp"
#include <boost/foreach.hpp>


#define POINTS_TOPIC "/velodynes/left/points"
#define CENTER_POINTS_TOPIC "/velodynes/center/points"
#define ROBOT_HEIGHT 2.8
#define POINT_TYPE pcl::PointXYZ

using namespace std::chrono_literals;


bool initiated = false;

void rescaleCloudsPrereq(pcl::PointCloud<pcl::PointXYZ>::Ptr& goldenCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& sampleCloud) {

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
    static Eigen::Matrix3f sampleEVs_Dir = pcaSample.getEigenVectors();
    static Eigen::Vector4f sampleMidPt = pcaSample.getMean();

    Eigen::Matrix4f sampleTransform = Eigen::Matrix4f::Identity();
    sampleTransform.block<3, 3>(0, 0) = sampleEVs_Dir;
    sampleTransform.block<4, 1>(0, 3) = sampleMidPt;
    pcl::PointCloud<pcl::PointXYZ>::Ptr orientedSample(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*sampleCloud, *orientedSample, sampleTransform.inverse());

}

void rescaleClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr& sampleCloud) {
    double xScale = 2.0;
    double yScale = 2.0;
    double zScale = 2.0;

    /*TODO move these two out of this scope for optimization*/
    Eigen::Matrix4f sampleTransform = Eigen::Matrix4f::Identity();
    pcl::PointCloud<pcl::PointXYZ>::Ptr orientedSample(new pcl::PointCloud<pcl::PointXYZ>);
    /*TODO move these two up for optimization*/

    pcl::transformPointCloud(*sampleCloud, *orientedSample, sampleTransform.inverse());

    BOOST_FOREACH(pcl::PointXYZ &point, orientedSample->points) {
      point.x = point.x / xScale;
      point.y = point.y / yScale;
      point.z = point.z / zScale;
    }

    pcl::transformPointCloud(*orientedSample, *sampleCloud, sampleTransform);

    // TODO filter the sample cloud
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

      rescaled_points_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("velodynes/center/rescaled_points", 10);
      rectangle_conditional_filter.setInputCloud(cloud);

     // RectangleFilter::RectangleFilter<POINT_TYPE>::Ptr rect_filter_condition (new RectangleFilter::RectangleFilter<POINT_TYPE>());

      pcl::ConditionOr<POINT_TYPE>::Ptr total_cond (new pcl::ConditionOr<POINT_TYPE>());
      total_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("x", pcl::ComparisonOps::GT, 1.0)));
      total_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("x", pcl::ComparisonOps::LT, -0.8)));
      total_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("y", pcl::ComparisonOps::GT, 0.8)));
      total_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("y", pcl::ComparisonOps::LT, -0.8)));
      // pcl::RectangleFilter<POINT_TYPE>::Ptr rect_filter_condition (new pcl::RectangleFilter<POINT_TYPE>());
      rectangle_conditional_filter.setCondition(total_cond);
      // TODO               add condition here ^^^
      grid.setLeafSize(1.15, 1.15, 1.15);
    }

  private:
    void pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      pcl::fromROSMsg(*msg, *cloud);
      grid.filter(*cloud);
      if (!initiated) {
        initiated = true;
        if ((cloud->size() > 0) && (center_cloud->size() > 0)) {
          rescaleCloudsPrereq(cloud, center_cloud);
        }
      }
    }

    void center_pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      pcl::fromROSMsg(*msg, *center_cloud);
      rescaleClouds(center_cloud);
      rectangle_conditional_filter.filter(*center_cloud);
      sensor_msgs::msg::PointCloud2 rescaled_points_msg;
      pcl::toROSMsg(*center_cloud, rescaled_points_msg);
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

    pcl::ConditionalRemoval<POINT_TYPE> rectangle_conditional_filter = pcl::ConditionalRemoval<POINT_TYPE>();
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr center_pcl_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rescaled_points_publisher;


    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PclFilter>());
  rclcpp::shutdown();
  return 0;
}
