#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "gazebo_msgs/msg/model_states.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "midterm_2024_msg/msg/midterm.hpp"
#include "mymat.hpp"
#include "myQuaternion.hpp"
#include "geometry_msgs/msg/wrench.hpp"

using std::placeholders::_1;

class Sun_generator : public rclcpp::Node
{
public:
    Sun_generator(const char* nodeName)
    : Node(nodeName)
    {
        //////////////// TODO ////////////////
        // TODO: you can set your subscriber and publishers
        // TODO: subscribed topic should be "demo/model_states_demo"
        // TODO: published topic should be "/Midterm/ForceInput/TorqueBySun" with type : geometry_msgs::msg::Wrench
        // TODO: published topic should be "/Midterm/recordings" with type : midterm_2024_msg::msg::Midterm

        //////////////// TODO End ////////////////
    }
private:

    void topic_callback(const gazebo_msgs::msg::ModelStates::SharedPtr msg) const
    {
        //////////////// TODO ////////////////
        // TODO: Calculate the torque by sun in world frame.
        // TODO: get information (position, quaternion, angular velocity)publish them by topic (in GCS)

    }
    // TODO: you can set your subscriber and publishers

    //////////////// TODO End ////////////////
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Sun_generator>("sun_torque_generator_20230001"));
    rclcpp::shutdown();

    return 0;
}
