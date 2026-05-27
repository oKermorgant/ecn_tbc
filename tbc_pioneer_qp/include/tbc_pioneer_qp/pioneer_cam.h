#ifndef PIONEERCAM_H
#define PIONEERCAM_H

#ifdef FOR_IGNITION_FORTRESS
#include <ignition/transport/Node.hh>
#include <ignition/msgs.hh>
#else
#include <gz/transport/Node.hh>
#include <gz/msgs.hh>
#endif

using gz::msgs::Double;
using gz::msgs::Model;
using gz::msgs::Pose;
using gz::msgs::Image;
using gz::msgs::Vector3d;
using gz::transport::Node;

#include <csignal>
#include <visp/vpColVector.h>
#include <visp/vpVelocityTwistMatrix.h>
#include <visp/vpSubMatrix.h>
#include <visp/vpSubColVector.h>
#include <visp/vpFeaturePoint.h>
#include <ecn_common/color_detector.h>

namespace tbc_pioneer_qp
{

class PioneerCam
{
  static constexpr auto a1{0.1};
  static constexpr auto r1{0.3};
public:
  PioneerCam();

  inline void detectColor(int r, int g, int b)
  {
    detector.detectColor(r, g, b);
  }

  // send a velocity to the joints
  void sendCommand(const vpColVector &v);

  vpMatrix camJacobian() const;
  vpFeaturePoint imagePoint() const;

  // get the camera x-y visibility limits
  inline vpColVector getCamLimits() const
  {
      vpColVector l(2);
      l[0] = detector.xLim();
      l[1] = detector.yLim();
      return l;
  }

  bool ok();
  double wheelRadius() const {return radius;}
  double wheelBaseLine() const {return  baseline;}
  double wheelMaxW() const {return wmax;}
  vpColVector navCommand(double d, double lambdaV, double lambdaW) const;
  inline vpColVector joints() const {return q;}

  inline double tilt() const {return q[1];}

private:

  bool stop{false};
  inline void signalHandler(int signal)
  {
    if(signal == SIGINT || signal == SIGTERM)
    {
      sendCommand(vpColVector(4));
      syncAt(std::chrono::milliseconds(100));
      stop = true;
    }
  }
  void syncAt(std::chrono::milliseconds delay);

  void waitForConnection();

  ecn_common::ColorDetector detector;
  Node node;

  static constexpr uint n{4};
  double radius{0.095};
  double baseline{.331};
  double a3{0.1};
  double d3{0.3};

  // current joint states
  std::vector<std::string> joint_names;
  vpColVector q{2,1};

  void jsCallback(const Model &msg);

  // positions
  vpHomogeneousMatrix wMr;  // robot
  vpHomogeneousMatrix wMc;  // cam frame
  vpTranslationVector wTt, wTb; // target / ball
  void poseCallback(const Pose &msg);

  // image
  void imageCallback(const Image &msg);

  // command
  double wmax{4.};
  vpMatrix Vmap;
  std::array<Node::Publisher, n> cmd_pub;
};
}

#endif
