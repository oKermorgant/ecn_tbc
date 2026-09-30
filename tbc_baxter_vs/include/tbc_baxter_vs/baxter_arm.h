#ifndef BAXTERARM_H
#define BAXTERARM_H

#include <rclcpp/node.hpp>
#include <baxter_core_msgs/msg/joint_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <image_transport/image_transport.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpVelocityTwistMatrix.h>
#include <cv_bridge/cv_bridge.hpp>
#include <ecn_common/color_detector.h>
#include <ctime>

namespace visp{}
using namespace visp;


inline double h_weight(double s, double s_act, double s_max)
{
  if(s_act > s_max)
    return h_weight(-s, -s_act, -s_max);

  if(s < s_act)
    return 0;

  return (s-s_act) / (s_max - s);
}


class BaxterArm
{

  using JointCommand = baxter_core_msgs::msg::JointCommand;
  using JointState = sensor_msgs::msg::JointState;
  using Image = sensor_msgs::msg::Image;
  using Float64MultiArray = std_msgs::msg::Float64MultiArray;

public:

  BaxterArm(std::string side = "right", bool sim = true);

  // run this control loop
  void useControl(const std::function<void ()>& callback, std::chrono::milliseconds dt);

  // joint space I/O
  vpColVector jointPosition() {return q_;}
  void setJointVelocity(vpColVector _qdot);

  // default arm position
  void home();

  void plot(const vpColVector& err);

  void setCameraVelocity(const vpColVector& _velocity);
  vpHomogeneousMatrix cameraPose();   // aka camera -> base bMc

  // Jacobian in camera frame
  vpMatrix cameraJacobian(const vpColVector &_q) const ;
  inline vpMatrix cameraJacobian() const
  {
    return cameraJacobian(q_);
  }

  inline vpColVector jointMin() const {return q_min_;}
  inline vpColVector jointMax() const {return q_max_;}
  inline vpColVector velocityMax() const {return v_max_;}

  inline double lambda() const {return lambda_;}
  inline auto use_feed_forward() const {return use_ff;}

  // camera part
  double x() {return cd_.x();}
  double y() {return cd_.y();}
  double area()  {return cd_.area();}
  double area_d() const {return area_d_;}


protected:
  // ROS  
  rclcpp::Node::SharedPtr node_;

  // joints
  rclcpp::Subscription<JointState>::SharedPtr joint_sub;
  std::vector<std::string> names_;

  // cmd
  rclcpp::Publisher<JointCommand>::SharedPtr cmd_pub;
  JointCommand cmd;

  // gain tuning
  double lambda_{1.1};
  bool use_ff{false};

  // online feedback
  rclcpp::Publisher<Float64MultiArray>::SharedPtr error_pub;

  vpColVector q_;
  vpHomogeneousMatrix wMc_, bMf_;
  vpVelocityTwistMatrix cWw_, fRRb_;

  // some checks
  bool lefty_, is_init_ = false, im_ok = false, js_ok = false;
  double area_d_ =  0.05;    // simulation value

  // image
  image_transport::ImageTransport im_tr;
  image_transport::Subscriber image_sub;
  image_transport::Publisher image_pub;
  ecn_common::ColorDetector cd_;
  int lost_count = 0;

  // joint limits
  vpColVector q_min_, q_max_, v_max_;

  // internal modeling
  // Direct Kinematic Model   // aka wrist -> fixed fMw
  vpHomogeneousMatrix fMw(const vpColVector &_q) const;
  // Classical Jacobian of wrist frame
  vpMatrix fJw(const vpColVector &_q) const;


};



#endif // BAXTERARM_H
