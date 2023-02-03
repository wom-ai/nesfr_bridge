#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <sys/ipc.h>
#include <sys/shm.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class NesfrRosBridge : public rclcpp::Node
{
public:
  NesfrRosBridge()
  : Node("nesfr_ros_bridge")
  {
    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = this->create_wall_timer(
        1ms, std::bind(&NesfrRosBridge::updateOdometry, this));

    // ftok to generate unique key
    key_t odometry_shm_key = ftok("/tmp/odometry_shm_file",65);
    key_t cmdvel_shm_key = ftok("/tmp/cmdvel_shm_file",65);
    if(odometry_shm_key < 0 || cmdvel_shm_key < 0){
      RCLCPP_ERROR(this->get_logger(), "Shared memory not found. Please make sure NESFR System is running...");
      exit(-1);
    }

    _last_read_timestamp = std::chrono::system_clock::now().time_since_epoch().count();


    // shmget returns an identifier in shmid
    int odometry_shmid = shmget(odometry_shm_key,1024,0666|IPC_CREAT);
    int cmdvel_shmid = shmget(cmdvel_shm_key,1024,0666|IPC_CREAT);

    // shmat to attach to shared memory
    _odometry_shm = static_cast<float *>(shmat(odometry_shmid,(void*)0,0));
    _odometry_ts = reinterpret_cast<uint64_t *>(_odometry_shm);
    _cmdvel_shm = static_cast<float *>(shmat(cmdvel_shmid,(void*)0,0));
    _cmdvel_ts = reinterpret_cast<uint64_t *>(_cmdvel_shm);

    cmdvel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 1, std::bind(&NesfrRosBridge::cmdvelCallback, this, std::placeholders::_1));
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);

  }

private:
  void updateOdometry()
  {
    if(_odometry_ts[0] <= _last_read_timestamp)
      return;
    _last_read_timestamp = _odometry_ts[0];

    auto current_time = this->get_clock()->now();

    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = current_time;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    t.transform.translation.x = _odometry_shm[2];
    t.transform.translation.y = _odometry_shm[3];
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, _odometry_shm[4]);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    std::cout << _odometry_shm[2] << " " << _odometry_shm[3] << " " << _odometry_shm[4] << std::endl;

    // Send the transformation
    tf_broadcaster_->sendTransform(t);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = _odometry_shm[2];
    odom.pose.pose.position.y = _odometry_shm[3];
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = _odometry_shm[5];
    odom.twist.twist.linear.y = _odometry_shm[6];
    odom.twist.twist.angular.z = _odometry_shm[7];
    odom_pub_->publish(odom);
  }

  void cmdvelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) const
  {
    _cmdvel_shm[2] = msg->linear.x;
    _cmdvel_shm[3] = msg->linear.y;
    _cmdvel_shm[4] = msg->angular.z;
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdvel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  uint64_t _last_read_timestamp;
  float* _odometry_shm;
  float* _cmdvel_shm;
  uint64_t* _odometry_ts;
  uint64_t* _cmdvel_ts;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NesfrRosBridge>());
  rclcpp::shutdown();
  return 0;
}
