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

#define POINTS_TOPIC "/camera/aligned_depth_to_color/color/points"
//#define POINTS_TOPIC "/velodyne_points"
#define ROBOT_HEIGHT 1.7

using namespace std::chrono_literals;


class PclFilter : public rclcpp::Node
{
  public:
    PclFilter()
    : Node("pcl_filter"), count_(0)
    {
      pcl_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        POINTS_TOPIC, 10, std::bind(&PclFilter::pcl_callback, this, std::placeholders::_1));

      grid.setLeafSize(1.15, 1.15, 1.15);


      pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB>());
      z_obstacle_cond = range_cond;
      z_obstacle_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::Ptr (new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::GT, 0.07)));
      z_obstacle_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::Ptr (new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LT, ROBOT_HEIGHT)));
      spatial_obstacle_filter.setInputCloud(cloud);
      spatial_obstacle_filter.setCondition(z_obstacle_cond);

      pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr traversible_range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB>());
      z_traversible_cond = traversible_range_cond;
      z_traversible_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::Ptr (new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::GT, -0.15)));
      z_traversible_cond->addComparison(pcl::FieldComparison<pcl::PointXYZRGB>::Ptr (new pcl::FieldComparison<pcl::PointXYZRGB>("z", pcl::ComparisonOps::LT, 0.08)));
      spatial_traversible_filter.setInputCloud(cloud);
      spatial_traversible_filter.setCondition(z_traversible_cond);

      /*
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

      timer_ = this->create_wall_timer(
          5ms, std::bind(&PclFilter::timer_callback, this));
      non_traversible_points_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/non_traversible_points", 10);
      traversible_points_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/traversible_points", 10);*/
    }

  private:
    void pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      // This will convert the message into a pcl::PointCloud
      pcl::fromROSMsg(*msg, *cloud);
      grid.filter(*cloud);

      //auto obstacle_filter = []() {
        sensor_msgs::msg::PointCloud2 obstacles_msg;
        spatial_obstacle_filter.filter(*cloud);
        pcl::toROSMsg(*cloud, obstacles_msg);
        traversible_points_publisher->publish(obstacles_msg);
        //}
    }
  /*
    void timer_callback() { }
  */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> grid = pcl::ApproximateVoxelGrid<pcl::PointXYZRGB>();


    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr z_obstacle_cond;
    pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr z_traversible_cond;
    pcl::ConditionalRemoval<pcl::PointXYZRGB> spatial_obstacle_filter = pcl::ConditionalRemoval<pcl::PointXYZRGB>();
    pcl::ConditionalRemoval<pcl::PointXYZRGB> spatial_traversible_filter = pcl::ConditionalRemoval<pcl::PointXYZRGB>();


    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr non_traversible_points_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr traversible_points_publisher;

  /*
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;*/
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
