#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
//#include "pcl.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/approximate_voxel_grid.h"

#define POINTS_TOPIC "/camera/aligned_depth_to_color/color/points"


using namespace std::chrono_literals;
//using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class PclFilter : public rclcpp::Node
{
  public:
    PclFilter()
    : Node("pcl_filtering"), count_(0)
    {
      pcl_subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        POINTS_TOPIC, 10, std::bind(&PclFilter::pcl_callback, this, std::placeholders::_1));

      traversible_points_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/traversible_points", 10);
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
      5ms, std::bind(&PclFilter::timer_callback, this));
    }

  private:
    void pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      // This will convert the message into a pcl::PointCloud
      pcl::fromROSMsg(*msg, *cloud);

      sensor_msgs::msg::PointCloud2 send_msg;
      pcl::toROSMsg(*cloud, send_msg);
      traversible_points_publisher->publish(send_msg);
    }

    void timer_callback()
    {
    }


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> grid = pcl::ApproximateVoxelGrid<pcl::PointXYZRGB>();
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr traversible_points_publisher;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
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
