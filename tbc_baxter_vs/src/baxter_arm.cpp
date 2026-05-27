#include <tbc_baxter_vs/baxter_arm.h>
#include <urdf/model.h>
#include <opencv2/highgui.hpp>
#include <thread>
#include <rclcpp/parameter_client.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

using namespace std;

BaxterArm::BaxterArm(std::string _side, bool sim) :
    node_{std::make_shared<rclcpp::Node>("control")},
    im_tr{node_}
{  
  node_->set_parameter(rclcpp::Parameter("use_sim_time", sim));


  // add control gain lambda as parameter
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.name = "lambda";
  descriptor.floating_point_range = {rcl_interfaces::msg::FloatingPointRange()
                                         .set__from_value(0.0)
                                         .set__to_value(10.0)};
  lambda_ = node_->declare_parameter(descriptor.name, lambda_, descriptor);
  use_ff = node_->declare_parameter("feedforward", false);

  static auto param_cb = node_->add_on_set_parameters_callback([this](const
                                                                      std::vector<rclcpp::Parameter> &params){

    for(const auto &param: params)
    {
      if(param.get_name() == "lambda")
        lambda_ = param.as_double();
      if(param.get_name() == "feedforward")
        use_ff = param.as_bool();
    }
    return rcl_interfaces::msg::SetParametersResult().set__successful(true);
  });


  // in case of misspell
  if (_side != "left")
    _side = "right";

  // which arm
  lefty_ = (_side == "left");

  // we detect green by default (sim)
  if(sim)
    cd_.detectColor(0, 255, 0);
  else if(lefty_)
    cd_.detectColor(255,0,0);
  else
    cd_.detectColor(0,255,0);
  cd_.fitCircle();

  cd_.setSaturationValue(100, 60);
  //if(!sim)
    cd_.showSegmentation();


  std::cout << "BaxterArm initialized for " << _side << " arm ";
  if(sim)
    std::cout << "and in simulation\n";
  else
    std::cout << "on the real robot\n";

  // joint space dimension: 7
  q_.resize(7);

  // init joint URDF names
  cmd.names.resize(7);
  cmd.names[0] = _side + "_s0";
  cmd.names[1] = _side + "_s1";
  cmd.names[2] = _side + "_e0";
  cmd.names[3] = _side + "_e1";
  cmd.names[4] = _side + "_w0";
  cmd.names[5] = _side + "_w1";
  cmd.names[6] = _side + "_w2";
  cmd.command.resize(7);

  // load Baxter description from robot_state_publisher
  const auto rsp_node(std::make_shared<rclcpp::Node>("baxter_rsp"));
  const auto rsp_param_srv = std::make_shared<rclcpp::SyncParametersClient>
      (rsp_node, "/robot/robot_state_publisher");
  rsp_param_srv->wait_for_service();
  if(!rsp_param_srv->has_parameter("robot_description"))
  {
    // cannot get the model anyway
    RCLCPP_WARN(node_->get_logger(), "cannot get Baxter model");
    return;
  }
  // init joint limits
  // parse URDF to get robot data (name, DOF, joint limits, etc.)
  urdf::Model model;
  model.initString(rsp_param_srv->get_parameter<string>("robot_description"));

  q_min_.resize(7);
  q_max_.resize(7);
  v_max_.resize(7);
  for(auto& joint: model.joints_)
  {
    for(unsigned int i=0;i<7;++i)
    {
      if(joint.second->name == cmd.names[i])
      {
        v_max_[i] = joint.second->limits->velocity;
        if(!sim)
          v_max_[i] *= .5;
        q_min_[i] = joint.second->limits->lower;
        q_max_[i] = joint.second->limits->upper;
      }
    }
  }

  // init fixed matrices
  // between wrist Fw and camera Fc
  wMc_[0][0] = 0;
  wMc_[0][1] = 1;
  wMc_[0][2] = 0;
  wMc_[0][3] = 0.03825;
  wMc_[1][0] = -1;
  wMc_[1][1] = 0;
  wMc_[1][2] = 0;
  wMc_[1][3] = 0.012;
  wMc_[2][0] = 0;
  wMc_[2][1] = 0;
  wMc_[2][2] = 1;
  wMc_[2][3] = 0.128905;

  cWw_.buildFrom(wMc_.inverse());

  // between Baxter base Fb and root frame of the arm Ff
  if(lefty_)
  {
    bMf_[0][0] = 0.707106781186548;
    bMf_[0][1] = -0.707106781186548;
    bMf_[0][2] = 0;
    bMf_[0][3] = 0.024645;
    bMf_[1][0] = 0.707106781186548;
    bMf_[1][1] = 0.707106781186548;
    bMf_[1][2] = 0;
    bMf_[1][3] = 0.219645;
    bMf_[2][0] = 0;
    bMf_[2][1] = 0;
    bMf_[2][2] = 1;
    bMf_[2][3] = 0.118588;
  }
  else
  {
    bMf_[0][0] = 0.707106781186548;
    bMf_[0][1] = 0.707106781186548;
    bMf_[0][2] = 0;
    bMf_[0][3] = 0.024645;
    bMf_[1][0] = -0.707106781186548;
    bMf_[1][1] = 0.707106781186548;
    bMf_[1][2] = 0;
    bMf_[1][3] = -0.219645;
    bMf_[2][0] = 0;
    bMf_[2][1] = 0;
    bMf_[2][2] = 1;
    bMf_[2][3] = 0.118588;
  }

  vpRotationMatrix bRf;
  bMf_.extract(bRf);
  fRRb_.buildFrom(vpTranslationVector(), bRf.inverse());    // this is just the frame change matrix [[R 0][0 R]]

  if(sim)
  {
    area_d_ = 0.08;
    // simulated camera parameters
    cd_.setCamera(640, 480, ecn_common::Deg(90));
  }
  else
  {
    // desired area in this case
    area_d_ = 0.03;

    // camera parameters
    if(lefty_)
      cd_.setCamera(403.33,403.33,336.04,208.45);
    else
      cd_.setCamera(404.38,404.38,323.58,196.39);

    // publisher to Baxter image
    image_pub = im_tr.advertise("/robot/xdisplay", 100);
  }

  // publisher to joint command
  cmd_pub = node_->create_publisher<JointCommand>("/robot/limb/"+_side+"/joint_command", 100);

  // subscriber to joint states
  joint_sub = node_->create_subscription<JointState>("/robot/joint_states", 1000, [&](const JointState::SharedPtr msg)
                                                     {
                                                       size_t idx{};
                                                       for(auto &name: msg->name)
                                                       {
                                                         for(unsigned int j=0;j<7;++j)
                                                         {
                                                           if(name == cmd.names[j])
                                                           {
                                                             q_[j] = msg->position[idx];
                                                             continue;
                                                           }
                                                         }
                                                         idx++;
                                                       }
                                                     });

  // publisher for visual error
  error_pub = node_->create_publisher<Float64MultiArray>("/" + _side + "/error", 1);

  // set image to None, subscriber instantiated in the image setter
  image_sub = im_tr.subscribe("/cameras/"+_side+"_hand_camera/image", 1, [&](const Image::ConstSharedPtr msg)
                              {
                                if(!is_init_)
                                  return;

                                // process with color detector
                                cv::Mat im_out;
                                auto im{cv_bridge::toCvCopy(msg)};

                                const auto detected = cd_.process(im->image, im_out);

                                im_ok = detected || im_ok;

                                if(detected && cd_.area() > 0.002)
                                  lost_count = 0;
                                else
                                  lost_count++;

                                // add setpoint
                                cv::circle(im_out, cv::Point(cd_.cam.u0, cd_.cam.v0),
                                           int(sqrt(area_d_*cd_.cam.px*cd_.cam.py/M_PI)),
                                           cv::Scalar(0,255,0), 2);

                                // show image
                                cv::imshow("Baxter", im_out);
                              });

  home();
}

void BaxterArm::home()
{
  cmd.command[0] = .24;
  cmd.command[1] = -1.15;
  cmd.command[2] = 0;
  cmd.command[3] = 1.64;
  cmd.command[4] = 0;
  cmd.command[5] = -.55;
  cmd.command[6] = 0;

  cmd.mode = cmd.POSITION_MODE;
  cmd_pub->publish(cmd);
  is_init_ = true;
  std::this_thread::sleep_for(1s);
}


void BaxterArm::setJointVelocity(vpColVector _qdot)
{
  if(is_init_ && lost_count > 10)
  {
    std::cout << "Object lost, going back to home position" << std::endl;
    home();
    return;
  }

  // saturate velocity before publishing
  for(unsigned int i=0;i<7;++i)
    cmd.command[i] = std::clamp(_qdot[i], -v_max_[i], v_max_[i]);
  cmd.mode = cmd.VELOCITY_MODE;
  cmd_pub->publish(cmd);
}

vpHomogeneousMatrix BaxterArm::cameraPose()
{
  return bMf_ * fMw(q_) * wMc_;  //  bMc
}

void BaxterArm::setCameraVelocity(const vpColVector &_velocity)
{
  setJointVelocity(cameraJacobian(q_).pseudoInverse() * _velocity);   // to joint velocity
}


/**
 * @brief get Jacobian expressed in camera frame
 */
vpMatrix BaxterArm::cameraJacobian(const vpColVector &_q) const
{
  // Jacobian in root frame fJw
  const auto J{fJw(_q)};

  // build wRRf, vector transform between Ff and Fw
  const auto wRf{fMw(_q).getRotationMatrix().inverse()};
  const vpVelocityTwistMatrix wRRf(vpTranslationVector(), wRf);
  return cWw_ * wRRf * J; // cJc = cWw * wRRf * fJw
}

/**
 * @brief direct geometric model (base to wrist <side>_hand_camera).
 */
vpHomogeneousMatrix BaxterArm::fMw(const vpColVector &_q) const
{
  vpHomogeneousMatrix M;
  const double c1 = cos(_q[0]);
  const double c2 = cos(_q[1]);
  const double c3 = cos(_q[2]);
  const double c4 = cos(_q[3]);
  const double c5 = cos(_q[4]);
  const double c6 = cos(_q[5]);
  const double c7 = cos(_q[6]);
  const double s1 = sin(_q[0]);
  const double s2 = sin(_q[1]);
  const double s3 = sin(_q[2]);
  const double s4 = sin(_q[3]);
  const double s5 = sin(_q[4]);
  const double s6 = sin(_q[5]);
  const double s7 = sin(_q[6]);
  if(lefty_)
  {
    M[0][0] = -((((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 + (s1*c3 - s2*s3*c1)*s5)*c6 - ((s1*s3 + s2*c1*c3)*s4 - c1*c2*c4)*s6)*c7 + (((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*s5 - (s1*c3 - s2*s3*c1)*c5)*s7;
    M[0][1] = ((((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 + (s1*c3 - s2*s3*c1)*s5)*c6 - ((s1*s3 + s2*c1*c3)*s4 - c1*c2*c4)*s6)*s7 + (((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*s5 - (s1*c3 - s2*s3*c1)*c5)*c7;
    M[0][2] = -(((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 + (s1*c3 - s2*s3*c1)*s5)*s6 - ((s1*s3 + s2*c1*c3)*s4 - c1*c2*c4)*c6;
    M[0][3] = -0.115975*(((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 + (s1*c3 - s2*s3*c1)*s5)*s6 - 0.115975*((s1*s3 + s2*c1*c3)*s4 - c1*c2*c4)*c6 - 0.01*((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 - 0.37429*(s1*s3 + s2*c1*c3)*s4 - 0.01*(s1*c3 - s2*s3*c1)*s5 - 0.069*s1*s3 - 0.069*s2*c1*c3 + 0.37429*c1*c2*c4 + 0.36442*c1*c2 + 0.069*c1 + 0.055695;
    M[1][0] = -((((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 - (s1*s2*s3 + c1*c3)*s5)*c6 - ((s1*s2*c3 - s3*c1)*s4 - s1*c2*c4)*s6)*c7 + (((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*s5 + (s1*s2*s3 + c1*c3)*c5)*s7;
    M[1][1] = ((((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 - (s1*s2*s3 + c1*c3)*s5)*c6 - ((s1*s2*c3 - s3*c1)*s4 - s1*c2*c4)*s6)*s7 + (((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*s5 + (s1*s2*s3 + c1*c3)*c5)*c7;
    M[1][2] = -(((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 - (s1*s2*s3 + c1*c3)*s5)*s6 - ((s1*s2*c3 - s3*c1)*s4 - s1*c2*c4)*c6;
    M[1][3] = -0.115975*(((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 - (s1*s2*s3 + c1*c3)*s5)*s6 - 0.115975*((s1*s2*c3 - s3*c1)*s4 - s1*c2*c4)*c6 - 0.01*((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 + 0.01*(s1*s2*s3 + c1*c3)*s5 - 0.37429*(s1*s2*c3 - s3*c1)*s4 - 0.069*s1*s2*c3 + 0.37429*s1*c2*c4 + 0.36442*s1*c2 + 0.069*s1 + 0.069*s3*c1;
    M[2][0] = (((s2*s4 - c2*c3*c4)*c5 + s3*s5*c2)*c6 + (s2*c4 + s4*c2*c3)*s6)*c7 - ((s2*s4 - c2*c3*c4)*s5 - s3*c2*c5)*s7;
    M[2][1] = -(((s2*s4 - c2*c3*c4)*c5 + s3*s5*c2)*c6 + (s2*c4 + s4*c2*c3)*s6)*s7 - ((s2*s4 - c2*c3*c4)*s5 - s3*c2*c5)*c7;
    M[2][2] = ((s2*s4 - c2*c3*c4)*c5 + s3*s5*c2)*s6 - (s2*c4 + s4*c2*c3)*c6;
    M[2][3] = 0.115975*((s2*s4 - c2*c3*c4)*c5 + s3*s5*c2)*s6 + 0.01*(s2*s4 - c2*c3*c4)*c5 - 0.115975*(s2*c4 + s4*c2*c3)*c6 - 0.37429*s2*c4 - 0.36442*s2 + 0.01*s3*s5*c2 - 0.37429*s4*c2*c3 - 0.069*c2*c3 + 0.281388;
  }
  else
  {
    M[0][0] = -((((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 + (s1*c3 - s2*s3*c1)*s5)*c6 - ((s1*s3 + s2*c1*c3)*s4 - c1*c2*c4)*s6)*c7 + (((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*s5 - (s1*c3 - s2*s3*c1)*c5)*s7;
    M[0][1] = ((((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 + (s1*c3 - s2*s3*c1)*s5)*c6 - ((s1*s3 + s2*c1*c3)*s4 - c1*c2*c4)*s6)*s7 + (((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*s5 - (s1*c3 - s2*s3*c1)*c5)*c7;
    M[0][2] = -(((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 + (s1*c3 - s2*s3*c1)*s5)*s6 - ((s1*s3 + s2*c1*c3)*s4 - c1*c2*c4)*c6;
    M[0][3] = -0.115975*(((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 + (s1*c3 - s2*s3*c1)*s5)*s6 - 0.115975*((s1*s3 + s2*c1*c3)*s4 - c1*c2*c4)*c6 - 0.01*((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 - 0.37429*(s1*s3 + s2*c1*c3)*s4 - 0.01*(s1*c3 - s2*s3*c1)*s5 - 0.069*s1*s3 - 0.069*s2*c1*c3 + 0.37429*c1*c2*c4 + 0.36442*c1*c2 + 0.069*c1 + 0.055695;
    M[1][0] = -((((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 - (s1*s2*s3 + c1*c3)*s5)*c6 - ((s1*s2*c3 - s3*c1)*s4 - s1*c2*c4)*s6)*c7 + (((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*s5 + (s1*s2*s3 + c1*c3)*c5)*s7;
    M[1][1] = ((((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 - (s1*s2*s3 + c1*c3)*s5)*c6 - ((s1*s2*c3 - s3*c1)*s4 - s1*c2*c4)*s6)*s7 + (((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*s5 + (s1*s2*s3 + c1*c3)*c5)*c7;
    M[1][2] = -(((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 - (s1*s2*s3 + c1*c3)*s5)*s6 - ((s1*s2*c3 - s3*c1)*s4 - s1*c2*c4)*c6;
    M[1][3] = -0.115975*(((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 - (s1*s2*s3 + c1*c3)*s5)*s6 - 0.115975*((s1*s2*c3 - s3*c1)*s4 - s1*c2*c4)*c6 - 0.01*((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 + 0.01*(s1*s2*s3 + c1*c3)*s5 - 0.37429*(s1*s2*c3 - s3*c1)*s4 - 0.069*s1*s2*c3 + 0.37429*s1*c2*c4 + 0.36442*s1*c2 + 0.069*s1 + 0.069*s3*c1;
    M[2][0] = (((s2*s4 - c2*c3*c4)*c5 + s3*s5*c2)*c6 + (s2*c4 + s4*c2*c3)*s6)*c7 - ((s2*s4 - c2*c3*c4)*s5 - s3*c2*c5)*s7;
    M[2][1] = -(((s2*s4 - c2*c3*c4)*c5 + s3*s5*c2)*c6 + (s2*c4 + s4*c2*c3)*s6)*s7 - ((s2*s4 - c2*c3*c4)*s5 - s3*c2*c5)*c7;
    M[2][2] = ((s2*s4 - c2*c3*c4)*c5 + s3*s5*c2)*s6 - (s2*c4 + s4*c2*c3)*c6;
    M[2][3] = 0.115975*((s2*s4 - c2*c3*c4)*c5 + s3*s5*c2)*s6 + 0.01*(s2*s4 - c2*c3*c4)*c5 - 0.115975*(s2*c4 + s4*c2*c3)*c6 - 0.37429*s2*c4 - 0.36442*s2 + 0.01*s3*s5*c2 - 0.37429*s4*c2*c3 - 0.069*c2*c3 + 0.281388;
  }
  return M;
}


/**
 * @brief compute Jacobian (base to wrist left_hand_camera)
 */
vpMatrix BaxterArm::fJw(const vpColVector &_q) const
{
  vpMatrix J(6, 7);

  const double c1 = cos(_q[0]);
  const double c2 = cos(_q[1]);
  const double c3 = cos(_q[2]);
  const double c4 = cos(_q[3]);
  const double c5 = cos(_q[4]);
  const double c6 = cos(_q[5]);
  const double s1 = sin(_q[0]);
  const double s2 = sin(_q[1]);
  const double s3 = sin(_q[2]);
  const double s4 = sin(_q[3]);
  const double s5 = sin(_q[4]);
  const double s6 = sin(_q[5]);

  if(lefty_)
  {
    J[0][0] = 0.115975*(((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 - (s1*s2*s3 + c1*c3)*s5)*s6 + 0.115975*((s1*s2*c3 - s3*c1)*s4 - s1*c2*c4)*c6 + 0.01*((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 - 0.01*(s1*s2*s3 + c1*c3)*s5 + 0.37429*(s1*s2*c3 - s3*c1)*s4 + 0.069*s1*s2*c3 - 0.37429*s1*c2*c4 - 0.36442*s1*c2 - 0.069*s1 - 0.069*s3*c1;
    J[0][1] = -(-0.115975*((s2*s4 - c2*c3*c4)*c5 + s3*s5*c2)*s6 - 0.01*(s2*s4 - c2*c3*c4)*c5 + 0.115975*(s2*c4 + s4*c2*c3)*c6 + 0.37429*s2*c4 + 0.36442*s2 - 0.01*s3*s5*c2 + 0.37429*s4*c2*c3 + 0.069*c2*c3)*c1;
    J[0][2] = 0.115975*s1*s3*s5*s6 + 0.01*s1*s3*s5 - 0.115975*s1*s4*c3*c6 - 0.37429*s1*s4*c3 - 0.115975*s1*s6*c3*c4*c5 - 0.01*s1*c3*c4*c5 - 0.069*s1*c3 + 0.115975*s2*s3*s4*c1*c6 + 0.37429*s2*s3*s4*c1 + 0.115975*s2*s3*s6*c1*c4*c5 + 0.01*s2*s3*c1*c4*c5 + 0.069*s2*s3*c1 + 0.115975*s2*s5*s6*c1*c3 + 0.01*s2*s5*c1*c3;
    J[0][3] = 0.115975*s1*s3*s4*s6*c5 + 0.01*s1*s3*s4*c5 - 0.115975*s1*s3*c4*c6 - 0.37429*s1*s3*c4 + 0.115975*s2*s4*s6*c1*c3*c5 + 0.01*s2*s4*c1*c3*c5 - 0.115975*s2*c1*c3*c4*c6 - 0.37429*s2*c1*c3*c4 - 0.115975*s4*c1*c2*c6 - 0.37429*s4*c1*c2 - 0.115975*s6*c1*c2*c4*c5 - 0.01*c1*c2*c4*c5;
    //_J[0][4] = 0;
    J[0][5] = 0.115975*s1*s3*s4*s6 - 0.115975*s1*s3*c4*c5*c6 - 0.115975*s1*s5*c3*c6 + 0.115975*s2*s3*s5*c1*c6 + 0.115975*s2*s4*s6*c1*c3 - 0.115975*s2*c1*c3*c4*c5*c6 - 0.115975*s4*c1*c2*c5*c6 - 0.115975*s6*c1*c2*c4;
    //_J[0][6] = 0;
    J[1][0] = -0.115975*(((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 + (s1*c3 - s2*s3*c1)*s5)*s6 - 0.115975*((s1*s3 + s2*c1*c3)*s4 - c1*c2*c4)*c6 - 0.01*((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 - 0.37429*(s1*s3 + s2*c1*c3)*s4 - 0.01*(s1*c3 - s2*s3*c1)*s5 - 0.069*s1*s3 - 0.069*s2*c1*c3 + 0.37429*c1*c2*c4 + 0.36442*c1*c2 + 0.069*c1;
    J[1][1] = -(-0.115975*((s2*s4 - c2*c3*c4)*c5 + s3*s5*c2)*s6 - 0.01*(s2*s4 - c2*c3*c4)*c5 + 0.115975*(s2*c4 + s4*c2*c3)*c6 + 0.37429*s2*c4 + 0.36442*s2 - 0.01*s3*s5*c2 + 0.37429*s4*c2*c3 + 0.069*c2*c3)*s1;
    J[1][2] = 0.115975*s1*s2*s3*s4*c6 + 0.37429*s1*s2*s3*s4 + 0.115975*s1*s2*s3*s6*c4*c5 + 0.01*s1*s2*s3*c4*c5 + 0.069*s1*s2*s3 + 0.115975*s1*s2*s5*s6*c3 + 0.01*s1*s2*s5*c3 - 0.115975*s3*s5*s6*c1 - 0.01*s3*s5*c1 + 0.115975*s4*c1*c3*c6 + 0.37429*s4*c1*c3 + 0.115975*s6*c1*c3*c4*c5 + 0.01*c1*c3*c4*c5 + 0.069*c1*c3;
    J[1][3] = 0.115975*s1*s2*s4*s6*c3*c5 + 0.01*s1*s2*s4*c3*c5 - 0.115975*s1*s2*c3*c4*c6 - 0.37429*s1*s2*c3*c4 - 0.115975*s1*s4*c2*c6 - 0.37429*s1*s4*c2 - 0.115975*s1*s6*c2*c4*c5 - 0.01*s1*c2*c4*c5 - 0.115975*s3*s4*s6*c1*c5 - 0.01*s3*s4*c1*c5 + 0.115975*s3*c1*c4*c6 + 0.37429*s3*c1*c4;
    //_J[1][4] = 0;
    J[1][5] = 0.115975*s1*s2*s3*s5*c6 + 0.115975*s1*s2*s4*s6*c3 - 0.115975*s1*s2*c3*c4*c5*c6 - 0.115975*s1*s4*c2*c5*c6 - 0.115975*s1*s6*c2*c4 - 0.115975*s3*s4*s6*c1 + 0.115975*s3*c1*c4*c5*c6 + 0.115975*s5*c1*c3*c6;
    //_J[1][6] = 0;
    //_J[2][0] = 0;
    //_J[2][1] = 0;
    //_J[2][2] = 0;
    //_J[2][3] = 0;
    //_J[2][4] = 0;
    //_J[2][5] = 0;
    //_J[2][6] = 0;
    //_J[3][0] = 0;
    J[3][1] = -s1;
    J[3][2] = c1*c2;
    J[3][3] = -s1*c3 + s2*s3*c1;
    J[3][4] = -(s1*s3 + s2*c1*c3)*s4 + c1*c2*c4;
    J[3][5] = ((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*s5 - (s1*c3 - s2*s3*c1)*c5;
    J[3][6] = -(((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 + (s1*c3 - s2*s3*c1)*s5)*s6 - ((s1*s3 + s2*c1*c3)*s4 - c1*c2*c4)*c6;
    //_J[4][0] = 0;
    J[4][1] = c1;
    J[4][2] = s1*c2;
    J[4][3] = s1*s2*s3 + c1*c3;
    J[4][4] = -(s1*s2*c3 - s3*c1)*s4 + s1*c2*c4;
    J[4][5] = ((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*s5 + (s1*s2*s3 + c1*c3)*c5;
    J[4][6] = -(((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 - (s1*s2*s3 + c1*c3)*s5)*s6 - ((s1*s2*c3 - s3*c1)*s4 - s1*c2*c4)*c6;
    J[5][0] = 1;
    //_J[5][1] = 0;
    J[5][2] = -s2;
    J[5][3] = s3*c2;
    J[5][4] = -s2*c4 - s4*c2*c3;
    J[5][5] = -(s2*s4 - c2*c3*c4)*s5 + s3*c2*c5;
    J[5][6] = ((s2*s4 - c2*c3*c4)*c5 + s3*s5*c2)*s6 - (s2*c4 + s4*c2*c3)*c6;
  }
  else
  {
    J[0][0] = 0.115975*(((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 - (s1*s2*s3 + c1*c3)*s5)*s6 + 0.115975*((s1*s2*c3 - s3*c1)*s4 - s1*c2*c4)*c6 + 0.01*((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 - 0.01*(s1*s2*s3 + c1*c3)*s5 + 0.37429*(s1*s2*c3 - s3*c1)*s4 + 0.069*s1*s2*c3 - 0.37429*s1*c2*c4 - 0.36442*s1*c2 - 0.069*s1 - 0.069*s3*c1;
    J[0][1] = -(-0.115975*((s2*s4 - c2*c3*c4)*c5 + s3*s5*c2)*s6 - 0.01*(s2*s4 - c2*c3*c4)*c5 + 0.115975*(s2*c4 + s4*c2*c3)*c6 + 0.37429*s2*c4 + 0.36442*s2 - 0.01*s3*s5*c2 + 0.37429*s4*c2*c3 + 0.069*c2*c3)*c1;
    J[0][2] = 0.115975*s1*s3*s5*s6 + 0.01*s1*s3*s5 - 0.115975*s1*s4*c3*c6 - 0.37429*s1*s4*c3 - 0.115975*s1*s6*c3*c4*c5 - 0.01*s1*c3*c4*c5 - 0.069*s1*c3 + 0.115975*s2*s3*s4*c1*c6 + 0.37429*s2*s3*s4*c1 + 0.115975*s2*s3*s6*c1*c4*c5 + 0.01*s2*s3*c1*c4*c5 + 0.069*s2*s3*c1 + 0.115975*s2*s5*s6*c1*c3 + 0.01*s2*s5*c1*c3;
    J[0][3] = 0.115975*s1*s3*s4*s6*c5 + 0.01*s1*s3*s4*c5 - 0.115975*s1*s3*c4*c6 - 0.37429*s1*s3*c4 + 0.115975*s2*s4*s6*c1*c3*c5 + 0.01*s2*s4*c1*c3*c5 - 0.115975*s2*c1*c3*c4*c6 - 0.37429*s2*c1*c3*c4 - 0.115975*s4*c1*c2*c6 - 0.37429*s4*c1*c2 - 0.115975*s6*c1*c2*c4*c5 - 0.01*c1*c2*c4*c5;
    //_J[0][4] = 0;
    J[0][5] = 0.115975*s1*s3*s4*s6 - 0.115975*s1*s3*c4*c5*c6 - 0.115975*s1*s5*c3*c6 + 0.115975*s2*s3*s5*c1*c6 + 0.115975*s2*s4*s6*c1*c3 - 0.115975*s2*c1*c3*c4*c5*c6 - 0.115975*s4*c1*c2*c5*c6 - 0.115975*s6*c1*c2*c4;
    //_J[0][6] = 0;
    J[1][0] = -0.115975*(((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 + (s1*c3 - s2*s3*c1)*s5)*s6 - 0.115975*((s1*s3 + s2*c1*c3)*s4 - c1*c2*c4)*c6 - 0.01*((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 - 0.37429*(s1*s3 + s2*c1*c3)*s4 - 0.01*(s1*c3 - s2*s3*c1)*s5 - 0.069*s1*s3 - 0.069*s2*c1*c3 + 0.37429*c1*c2*c4 + 0.36442*c1*c2 + 0.069*c1;
    J[1][1] = -(-0.115975*((s2*s4 - c2*c3*c4)*c5 + s3*s5*c2)*s6 - 0.01*(s2*s4 - c2*c3*c4)*c5 + 0.115975*(s2*c4 + s4*c2*c3)*c6 + 0.37429*s2*c4 + 0.36442*s2 - 0.01*s3*s5*c2 + 0.37429*s4*c2*c3 + 0.069*c2*c3)*s1;
    J[1][2] = 0.115975*s1*s2*s3*s4*c6 + 0.37429*s1*s2*s3*s4 + 0.115975*s1*s2*s3*s6*c4*c5 + 0.01*s1*s2*s3*c4*c5 + 0.069*s1*s2*s3 + 0.115975*s1*s2*s5*s6*c3 + 0.01*s1*s2*s5*c3 - 0.115975*s3*s5*s6*c1 - 0.01*s3*s5*c1 + 0.115975*s4*c1*c3*c6 + 0.37429*s4*c1*c3 + 0.115975*s6*c1*c3*c4*c5 + 0.01*c1*c3*c4*c5 + 0.069*c1*c3;
    J[1][3] = 0.115975*s1*s2*s4*s6*c3*c5 + 0.01*s1*s2*s4*c3*c5 - 0.115975*s1*s2*c3*c4*c6 - 0.37429*s1*s2*c3*c4 - 0.115975*s1*s4*c2*c6 - 0.37429*s1*s4*c2 - 0.115975*s1*s6*c2*c4*c5 - 0.01*s1*c2*c4*c5 - 0.115975*s3*s4*s6*c1*c5 - 0.01*s3*s4*c1*c5 + 0.115975*s3*c1*c4*c6 + 0.37429*s3*c1*c4;
    //_J[1][4] = 0;
    J[1][5] = 0.115975*s1*s2*s3*s5*c6 + 0.115975*s1*s2*s4*s6*c3 - 0.115975*s1*s2*c3*c4*c5*c6 - 0.115975*s1*s4*c2*c5*c6 - 0.115975*s1*s6*c2*c4 - 0.115975*s3*s4*s6*c1 + 0.115975*s3*c1*c4*c5*c6 + 0.115975*s5*c1*c3*c6;
    //_J[1][6] = 0;
    //_J[2][0] = 0;
    //_J[2][1] = 0;
    //_J[2][2] = 0;
    //_J[2][3] = 0;
    //_J[2][4] = 0;
    //_J[2][5] = 0;
    //_J[2][6] = 0;
    //_J[3][0] = 0;
    J[3][1] = -s1;
    J[3][2] = c1*c2;
    J[3][3] = -s1*c3 + s2*s3*c1;
    J[3][4] = -(s1*s3 + s2*c1*c3)*s4 + c1*c2*c4;
    J[3][5] = ((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*s5 - (s1*c3 - s2*s3*c1)*c5;
    J[3][6] = -(((s1*s3 + s2*c1*c3)*c4 + s4*c1*c2)*c5 + (s1*c3 - s2*s3*c1)*s5)*s6 - ((s1*s3 + s2*c1*c3)*s4 - c1*c2*c4)*c6;
    //_J[4][0] = 0;
    J[4][1] = c1;
    J[4][2] = s1*c2;
    J[4][3] = s1*s2*s3 + c1*c3;
    J[4][4] = -(s1*s2*c3 - s3*c1)*s4 + s1*c2*c4;
    J[4][5] = ((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*s5 + (s1*s2*s3 + c1*c3)*c5;
    J[4][6] = -(((s1*s2*c3 - s3*c1)*c4 + s1*s4*c2)*c5 - (s1*s2*s3 + c1*c3)*s5)*s6 - ((s1*s2*c3 - s3*c1)*s4 - s1*c2*c4)*c6;
    J[5][0] = 1.;
    //_J[5][1] = 0;
    J[5][2] = -s2;
    J[5][3] = s3*c2;
    J[5][4] = -s2*c4 - s4*c2*c3;
    J[5][5] = -(s2*s4 - c2*c3*c4)*s5 + s3*c2*c5;
    J[5][6] = ((s2*s4 - c2*c3*c4)*c5 + s3*s5*c2)*s6 - (s2*c4 + s4*c2*c3)*c6;
  }
  return J;
}

void BaxterArm::plot(const vpColVector &err)
{
  static Float64MultiArray err_msg;
  err_msg.data = err.toStdVector();
  error_pub->publish(err_msg);
}

void BaxterArm::useControl(const std::function<void()> &callback, std::chrono::milliseconds dt)
{
  static auto timer = node_->create_wall_timer(dt, callback);
  rclcpp::spin(node_);
  rclcpp::shutdown();
}
