#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "coin_rolling_msg/msg/coin_rolling.hpp"


using std::placeholders::_1;

class MinimalPubSub : public rclcpp::Node
{
public:
    MinimalPubSub()
    : Node("inclined_coin_pubsub")
    {
        subscription_ = this->create_subscription<gazebo_msgs::msg::ModelStates> (
                "demo/model_states_demo",
                10,
                std::bind(&MinimalPubSub::topic_callback, this, _1)
                );

        publisher_ = this->create_publisher<coin_rolling_msg::msg::CoinRolling>("coin/inclined_coin",10);
    }
private:

    void topic_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg) const
    {

        auto message_pose = msg->pose[1];
        auto message_twist = msg->twist[1];

        geometry_msgs::msg::Vector3 message_momentum;

        //////////////// TODO ////////////////
        // Calculate the angular momentum vector of the rolling coin on the world frame
        // and publiseh the answer.

        double linx = msg->pose[1].position.x;
        double liny = msg->pose[1].position.y;
        double linz = msg->pose[1].position.z;

        double vx = msg->twist[1].linear.x;
        double vy = msg->twist[1].linear.y;
        double vz = msg->twist[1].linear.z;

        double ang_mom_center_x = mass_ * (liny * vz - linz * vy);
        double ang_mom_center_y = mass_ * (linz * vx - linx * vz);
        double ang_mom_center_z = mass_ * (linx * vy - liny * vx);

        double w = msg->pose[1].orientation.w;
        double x = msg->pose[1].orientation.x;
        double y = msg->pose[1].orientation.y;
        double z = msg->pose[1].orientation.z;

        double R11 = 1.0 - 2.0*y*y - 2.0*z*z;
        double R12 = 2.0*x*y - 2.0*w*z;
        double R13 = 2.0*x*z + 2.0*w*y;

        double R21 = 2.0*x*y + 2.0*w*z;
        double R22 = 1.0 - 2.0*x*x - 2.0*z*z;
        double R23 = 2.0*y*z - 2.0*w*x;

        double R31 = 2.0*x*z - 2.0*w*y;
        double R32 = 2.0*y*z + 2.0*w*x;
        double R33 = 1.0 - 2.0*x*x - 2.0*y*y;

        double wx = msg->twist[1].angular.x;
        double wy = msg->twist[1].angular.y;
        double wz = msg->twist[1].angular.z;

        double body_wx = R11 * wx + R21 * wy + R31 * wz;
        double body_wy = R12 * wx + R22 * wy + R32 * wz;
        double body_wz = R13 * wx + R23 * wy + R33 * wz;

        double ang_mom_body_in_body_frame_x = ixx_ * body_wx;
        double ang_mom_body_in_body_frame_y = iyy_ * body_wy;
        double ang_mom_body_in_body_frame_z = izz_ * body_wz;

        double ang_mom_body_x = R11 * ang_mom_body_in_body_frame_x + R12 * ang_mom_body_in_body_frame_y + R13 * ang_mom_body_in_body_frame_z;
        double ang_mom_body_y = R21 * ang_mom_body_in_body_frame_x + R22 * ang_mom_body_in_body_frame_y + R23 * ang_mom_body_in_body_frame_z;
        double ang_mom_body_z = R31 * ang_mom_body_in_body_frame_x + R32 * ang_mom_body_in_body_frame_y + R33 * ang_mom_body_in_body_frame_z;

        double ang_mom_x = ang_mom_center_x + ang_mom_body_x;
        double ang_mom_y = ang_mom_center_y + ang_mom_body_y;
        double ang_mom_z = ang_mom_center_z + ang_mom_body_z;

        message_momentum.x = ang_mom_x;
        message_momentum.y = ang_mom_y;
        message_momentum.z = ang_mom_z;

        // message_momentum.x = 0.0;
        // message_momentum.y = 0.0;
        // message_momentum.z = 0.0;

        //////////////// TODO End ////////////////

        auto message = coin_rolling_msg::msg::CoinRolling();
        message.angular_momentum = message_momentum;
        message.pose = message_pose;
        message.twist = message_twist;
        publisher_->publish(message);
    }

    rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr subscription_;
    rclcpp::Publisher<coin_rolling_msg::msg::CoinRolling>::SharedPtr publisher_;

    double mass_ = 10.0;
    double ixx_ = 0.7;
    double iyy_ = 0.7;
    double izz_ = 1.25;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPubSub>());
    rclcpp::shutdown();

    return 0;
}