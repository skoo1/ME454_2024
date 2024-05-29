#include <memory>
#include <cmath>
#include <random>
// PCL headers
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

#define MAX_SAMPLE 1024

class LidarSubsciber : public rclcpp::Node
{
  public:
    LidarSubsciber()
    : Node("lidar_process")
    {
      laser_subs_ = this->create_subscription<sensor_msgs::msg::LaserScan> ("/ray/laserscan", 10, std::bind(&LidarSubsciber::laser_callback, this, std::placeholders::_1));
      point_subs_ = this->create_subscription<sensor_msgs::msg::PointCloud> ("/ray/pointcloud", 10, std::bind(&LidarSubsciber::point_callback, this, std::placeholders::_1));
      pcl_viewer_ = new pcl::visualization::PCLVisualizer("Cloud Viewer");
      pcl_viewer_->setBackgroundColor(0, 0, 0);
      angle_min = 0.0;
      angle_max = 0.0;
      angle_inc = 0.0;
    }

  private:

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
      int curr_time = msg->header.stamp.sec * 1000 + msg->header.stamp.nanosec / 1000000; // current time in ms
      RCLCPP_INFO(this->get_logger(), "LaserScan time : %8d / \t Angle range : %4.3f ~ %4.3f (increment %4.3f)", curr_time, msg->angle_min, msg->angle_max, msg->angle_increment);
      
      if (n_smp == 0) n_smp = round((msg->angle_max - msg->angle_min) / msg->angle_increment) + 1;

      // TODO 1. storing the LaserScan message
      // member variables: angle_min, angle_max, angle_inc, ranges
    }

    void point_callback(const sensor_msgs::msg::PointCloud::SharedPtr msg)
    {
      int curr_time = msg->header.stamp.sec * 1000 + msg->header.stamp.nanosec / 1000000; // current time in ms
      RCLCPP_INFO(this->get_logger(), "PointCloud time : %8d", curr_time);
      
      // pointcloud from the PointCloud message
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a(new pcl::PointCloud<pcl::PointXYZ>);
      cloud_a->resize(n_smp);
      
      for (int i_pc = 0; i_pc < n_smp; i_pc++)
      {
        cloud_a->at(i_pc).x = msg->points[i_pc].x;
        cloud_a->at(i_pc).y = msg->points[i_pc].y;
        cloud_a->at(i_pc).z = msg->points[i_pc].z;
      }

      // TODO 2. pointcloud from the LaserScan message
      // 1. make a new pointcloud (cloud_b) 
      // 2. add a random Gaussian noise
      // 3. calculate points from ranges

      // TODO 3. pointcloud visualization

      pcl_viewer_->addPointCloud<pcl::PointXYZ>(cloud_a, "cloud_pc");
      
      pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "cloud_pc");
      pcl_viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR , 1.00, 1.00, 0.00, "cloud_pc");

      pcl_viewer_->spinOnce();
      pcl_viewer_->removePointCloud("cloud_pc");
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subs_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr point_subs_;
    pcl::visualization::PCLVisualizer *pcl_viewer_;
    int n_smp = 0;
    float angle_min, angle_max, angle_inc;
    float ranges[MAX_SAMPLE];

    // random number generator
    std::random_device rd {};
    std::mt19937 gen {rd()};
    std::normal_distribution<> distribution {0.0, 0.5};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarSubsciber>());
  rclcpp::shutdown();
  return 0;
}