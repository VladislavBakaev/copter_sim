#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>
#include "sensor_msgs/msg/imu.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <math.h>
#include <iostream>
#include <random>

using namespace std;
using std::placeholders::_1;

class CopterPlugin : public rclcpp::Node
{
    public:

        CopterPlugin() : Node("copter_plugin_node")
        {
            timer_velocity = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&CopterPlugin::velocityTimer, this));
            timer_position = this->create_wall_timer(
                std::chrono::milliseconds(100),
                std::bind(&CopterPlugin::positionControlTimer, this));
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
            subscription_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
                    "/cmd_vel", 10, std::bind(&CopterPlugin::cmdVelocityCallback, this, _1));
            
            // publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/lin_acc", 10);
            
            odom_tf.transform.translation.x = 0.0;
            odom_tf.transform.translation.y = 0.0;
            odom_tf.transform.translation.z = 0.0;
        }
    
    private:
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_vel_;
        rclcpp::TimerBase::SharedPtr timer_position;
        rclcpp::TimerBase::SharedPtr timer_velocity;
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        geometry_msgs::msg::TransformStamped odom_tf;
        // rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;

        double lin_vel_x = 0;
        double lin_vel_y = 0;
        double lin_vel_z = 0;

        double ang_vel_r = 0;
        double ang_vel_p = 0;
        double ang_vel_y = 0;

        double cur_lin_vel_x = 0;
        double cur_lin_vel_y = 0;
        double cur_lin_vel_z = 0;

        double fly_angle_r_max = 0.5;
        double fly_angle_p_max = 0.5;

        double fly_angle_r_koef = 0.3;
        double fly_angle_p_koef = 0.3;

        double const_time = 0.2;

        double last_iter = this->now().seconds();

        std::random_device rd{};
        std::mt19937 gen{rd()};
        std::normal_distribution<double> d{0.0,0.005};

        void positionControlTimer()
        {
            rclcpp::Time cur_time = this->now();
            double delta = cur_time.seconds()-last_iter;
            last_iter = cur_time.seconds();

            tf2::Quaternion quat(odom_tf.transform.rotation.x, odom_tf.transform.rotation.y, odom_tf.transform.rotation.z, odom_tf.transform.rotation.w);

            tf2::Vector3 rpy = convertQuaternion2RPY(quat);

            if (cur_lin_vel_y*fly_angle_r_koef > fly_angle_r_max){
                rpy[0] = -fly_angle_r_max;
            }
            else{
                rpy[0] = -cur_lin_vel_y*fly_angle_r_koef;
            }

            if (cur_lin_vel_x*fly_angle_p_koef > fly_angle_p_max){
                rpy[1] = fly_angle_p_max;
            }
            else{
                rpy[1] = cur_lin_vel_x*fly_angle_p_koef;
            }

            rpy[2] += ang_vel_y*delta;

            quat = convertRPY2Quaternion(rpy);
            // RCLCPP_INFO(this->get_logger(), "quaternion: '%f' '%f' '%f' '%f'", quat[0], quat[1], quat[2], quat[3]);

            odom_tf.transform.translation.x += cur_lin_vel_x*delta;
            odom_tf.transform.translation.y += cur_lin_vel_y*delta;
            odom_tf.transform.translation.z += cur_lin_vel_z*delta;
            odom_tf.transform.rotation.x = quat[0];
            odom_tf.transform.rotation.y = quat[1];
            odom_tf.transform.rotation.z = quat[2];
            odom_tf.transform.rotation.w = quat[3];
            odom_tf.header.frame_id = "world";
            odom_tf.child_frame_id = "copter_desired";
            odom_tf.header.stamp = cur_time;

            tf_broadcaster_->sendTransform(odom_tf);
        }

        void cmdVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
            lin_vel_x = msg->linear.x;
            lin_vel_y = msg->linear.y;
            lin_vel_z = msg->linear.z;

            ang_vel_y = msg->angular.z;
            // ang_vel_r = msg->angular.x;
            // ang_vel_p = msg->angular.y;
        }

        void velocityTimer(){
            if (std::abs(lin_vel_x - cur_lin_vel_x)>0.1){
                double delta = (lin_vel_x - cur_lin_vel_x)*const_time + d(gen);
                cur_lin_vel_x += delta;
            }
            else{
                cur_lin_vel_x = lin_vel_x;
            }

            if (std::abs(lin_vel_y - cur_lin_vel_y)>0.1){
                double delta = (lin_vel_y - cur_lin_vel_y)*const_time + d(gen);
                cur_lin_vel_y += delta;
            }
            else{
                cur_lin_vel_y = lin_vel_y;
            }

            if (std::abs(lin_vel_z - cur_lin_vel_z)>0.1){
                double delta = (lin_vel_z - cur_lin_vel_z)*const_time;
                cur_lin_vel_z += delta;
            }
            else{
                cur_lin_vel_z = lin_vel_z;
            }

            // RCLCPP_INFO(this->get_logger(), "x: '%f', y: '%f', z: '%f'", lin_x_acc, lin_y_acc, lin_z_acc);
        }

        tf2::Vector3 convertQuaternion2RPY(const tf2::Quaternion quaternion){
            tf2::Matrix3x3 matrix(quaternion);
            double roll, pitch, yaw;
            matrix.getRPY(roll, pitch, yaw);
            tf2::Vector3 rpy(roll, pitch, yaw);
            return rpy;
        }

        tf2::Quaternion convertRPY2Quaternion(const tf2::Vector3 rpy){
            tf2::Quaternion quat;
            quat.setRPY(rpy[0], rpy[1], rpy[2]);
            return quat;
        }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CopterPlugin>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}