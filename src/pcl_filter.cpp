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
#include "pcl/common/io.h"
#include "pcl/Vertices.h"
#include <pcl/point_cloud.h>
#include "pcl/common/transforms.h"

#define POINTS_TOPIC "/velodynes/left/points"
#define ROBOT_HEIGHT 0.8
#define POINT_TYPE pcl::PointXYZ

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


      pcl::ConditionAnd<POINT_TYPE>::Ptr range_cond (new pcl::ConditionAnd<POINT_TYPE>());
      z_obstacle_cond = range_cond;
      z_obstacle_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("z", pcl::ComparisonOps::LT, 0.27)));
      z_obstacle_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("z", pcl::ComparisonOps::GT, -ROBOT_HEIGHT)));

      // TODO verify these conditions after checking with final pcl
      z_obstacle_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("x", pcl::ComparisonOps::GT, -0.70)));
      // z_obstacle_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("x", pcl::ComparisonOps::LT, -0.5)));
      /*
      z_obstacle_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("y", pcl::ComparisonOps::GT, 0.5)));
      z_obstacle_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("y", pcl::ComparisonOps::LT, -0.5)));
      */

      const std::vector<pcl::Vertices> &polygon = {}; // = std::vector<pcl::Vertices>::Vector();
      pcl::Vertices v1, v2, v3, v4;
      spatial_obstacle_filter.setInputCloud(cloud);
      spatial_obstacle_filter.setCondition(z_obstacle_cond);
      pcl::ConditionAnd<POINT_TYPE>::Ptr traversible_range_cond (new pcl::ConditionAnd<POINT_TYPE>());
      z_traversible_cond = traversible_range_cond;
      z_traversible_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("z", pcl::ComparisonOps::GT, -0.05)));
      z_traversible_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("z", pcl::ComparisonOps::LT, 0.05)));

      spatial_traversible_filter.setInputCloud(traversible_cloud);
      spatial_traversible_filter.setCondition(z_traversible_cond);

      /*
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

      timer_ = this->create_wall_timer(
      5ms, std::bind(&PclFilter::timer_callback, this));*/
      non_traversible_points_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/non_traversible_points", 10);
      traversible_points_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/traversible_points", 10);
    }

  private:
    void pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      // This will convert the message into a pcl::PointCloud
      pcl::fromROSMsg(*msg, *cloud);
      grid.filter(*cloud);

      auto pcl_filter_lambda = [&](pcl::PointCloud<POINT_TYPE>::Ptr cloud, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, pcl::ConditionalRemoval<POINT_TYPE> filter) {
        sensor_msgs::msg::PointCloud2 obstacles_msg;
        // TODO transform coordinates here
	float theta = -0.687132; //M_PI/4; 
	Eigen::Affine3f transform_y = Eigen::Affine3f::Identity();
	transform_y.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<POINT_TYPE>());
        pcl::transformPointCloud(*cloud, *transformed_cloud, transform_y);
        static pcl::ConditionalRemoval<POINT_TYPE> tf_filter = pcl::ConditionalRemoval<POINT_TYPE>();
        pcl::ConditionOr<POINT_TYPE>::Ptr tf_filter_cond (new pcl::ConditionOr<POINT_TYPE>());
	tf_filter_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("x", pcl::ComparisonOps::GT, 0.8)));
	tf_filter_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("x", pcl::ComparisonOps::LT, -0.5)));
	tf_filter_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("y", pcl::ComparisonOps::GT, 2.5)));
	tf_filter_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("y", pcl::ComparisonOps::LT, -0.5)));
	tf_filter.setInputCloud(transformed_cloud);
	tf_filter.setCondition(tf_filter_cond);
	tf_filter.filter(*transformed_cloud);
	

	float inv_theta = -theta; 
	Eigen::Affine3f inv_transform_y = Eigen::Affine3f::Identity();
	inv_transform_y.rotate(Eigen::AngleAxisf(inv_theta, Eigen::Vector3f::UnitY()));
        pcl::PointCloud<pcl::PointXYZ>::Ptr inv_transformed_cloud (new pcl::PointCloud<POINT_TYPE>());
        pcl::transformPointCloud(*transformed_cloud, *inv_transformed_cloud, inv_transform_y);

	filter.setInputCloud(inv_transformed_cloud);
	filter.filter(*inv_transformed_cloud);
        pcl::toROSMsg(*inv_transformed_cloud, obstacles_msg);
        pub->publish(obstacles_msg);
      };

      // TODO add matrix transform here
      std::thread obstacles_publisher(pcl_filter_lambda, cloud, non_traversible_points_publisher, spatial_obstacle_filter);

      pcl::fromROSMsg(*msg, *traversible_cloud);
      grid.filter(*traversible_cloud);

      sensor_msgs::msg::PointCloud2 traversible_points_msg;
      spatial_traversible_filter.filter(*traversible_cloud);
      pcl::toROSMsg(*traversible_cloud, traversible_points_msg);
      traversible_points_publisher->publish(traversible_points_msg);
      //std::thread traversible_points_publisher(pcl_filter_lambda, *cloud, traversible_points_publisher, spatial_traversible_filter);

      obstacles_publisher.join();
      //traversible_points_publisher.join();
    }

    void timer_callback() { }

    pcl::PointCloud<POINT_TYPE>::Ptr cloud = boost::make_shared<pcl::PointCloud<POINT_TYPE>>();
    pcl::PointCloud<POINT_TYPE>::Ptr traversible_cloud = boost::make_shared<pcl::PointCloud<POINT_TYPE>>();
    pcl::ApproximateVoxelGrid<POINT_TYPE> grid = pcl::ApproximateVoxelGrid<POINT_TYPE>();

    pcl::ConditionAnd<POINT_TYPE>::Ptr z_obstacle_cond;
    pcl::ConditionAnd<POINT_TYPE>::Ptr z_traversible_cond;
    pcl::ConditionalRemoval<POINT_TYPE> spatial_obstacle_filter = pcl::ConditionalRemoval<POINT_TYPE>();
    pcl::ConditionalRemoval<POINT_TYPE> spatial_traversible_filter = pcl::ConditionalRemoval<POINT_TYPE>();


    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr non_traversible_points_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr traversible_points_publisher;


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
