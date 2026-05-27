#include <tbc_pioneer_qp/pioneer_cam.h>
#include <opencv2/imgproc.hpp>
#include <chrono>

using namespace std::chrono_literals;
using namespace tbc_pioneer_qp;
using namespace std;
using Clock = std::chrono::steady_clock;

inline vpTranslationVector toTranslation(const Vector3d &v)
{
  return {v.x(), v.y(), v.z()};
}

void PioneerCam::syncAt(std::chrono::milliseconds delay)
{
  static auto last{Clock::now()};
  std::this_thread::sleep_until(last + delay);
  last = Clock::now();
}

double toPi(double t)
{
  if(t > M_PI)
    return toPi(t - 2*M_PI);
  if(t <= -M_PI)
    return toPi(t + 2*M_PI);
  return t;
}

PioneerCam::PioneerCam()
{
  // joints
  joint_names = {"pan", "tilt", "left_wheel", "right_wheel"};
  for(size_t joint = 0; joint < n; ++joint)
    cmd_pub[joint] = node.Advertise<Double>("/pioneer/" + joint_names[joint] + "_cmd");
  node.Subscribe("/world/pioneer_world/model/pioneer/joint_state", &PioneerCam::jsCallback, this);

  // poses
  for(const string model: {"pioneer", "ball", "target"})
    node.Subscribe("/model/" + model + "/pose", &PioneerCam::poseCallback, this);

  // image
  node.Subscribe("/pioneer/camera", &PioneerCam::imageCallback, this);
  detector.detectColor(0,255,0);
  detector.setCamera(800, 600, ecn_common::Rad(1.5));
  detector.fitCircle();
  detector.showOutput();
  waitForConnection();

  // command
  // map between control vector u (v, w, p, t) and actual joints (p, t, wl, wr)
  Vmap.resize(4, 4);
  Vmap[0][2] = 1;
  Vmap[1][3] = -1;
  Vmap[2][0] = Vmap[3][0] = 1/radius;
  Vmap[2][1] = -(Vmap[3][1] = baseline/radius);
}


bool PioneerCam::ok()
{
  syncAt(100ms);
  return !stop;
}

void PioneerCam::waitForConnection()
{

  std::cout << "Waiting for simulation info..." << std::endl;
  while(!stop && (q.getRows() == 0 ||
                   wTt.frobeniusNorm() < 1e-3 ||
                   wTb.frobeniusNorm() < 1e-3 ||
                   wMr.getTranslationVector().frobeniusNorm() < 1e-3))
  {
    syncAt(50ms);
  }
  this_thread::sleep_for(1s);
}

void PioneerCam::sendCommand(const vpColVector &u)
{
  auto qdot{Vmap*u};

  //std::cout << "Sending " << u.t() << " -> " << qdot.t() << std::endl;

  // ensure bounded wheel velocities
  if(const auto scale{std::max(std::abs(qdot[2])/wmax, std::abs(qdot[3])/wmax)}; scale > 1.)
    qdot /= scale;

  // ensure joint range for tilt
  if(std::abs(q[1]) > 3*M_PI/8)
    qdot[1] = -0.1*q[1];

  for(size_t i = 0; i < n; ++i)
  {
    Double msg;
    msg.set_data(qdot[i]);
    cmd_pub[i].Publish(msg);
  }
}

vpMatrix PioneerCam::camJacobian() const
{
  vpMatrix J(6, 4);  
  const double c1 = cos(q[0]);
  const double c2 = cos(q[1]);
  const double s1 = sin(q[0]);
  const double s2 = sin(q[1]);
  J[0][0] = s1;
  J[0][1] = -a1*c1;
  //J[0][2] = 0;
  //J[0][3] = 0;
  J[1][0] = -s2*c1;
  J[1][1] = -a1*s1*s2;
  //J[1][2] = 0;
  //J[1][3] = 0;
  J[2][0] = c1*c2;
  J[2][1] = a1*s1*c2;
  //J[2][2] = 0;
  //J[2][3] = 0;
  //J[3][0] = 0;
  //J[3][1] = 0;
  //J[3][2] = 0;
  J[3][3] = 1.;
  //J[4][0] = 0;
  J[4][1] = -c2;
  J[4][2] = -c2;
  //J[4][3] = 0;
  //J[5][0] = 0;
  J[5][1] = -s2;
  J[5][2] = -s2;
  //J[5][3] = 0;
  return J;
}

vpFeaturePoint PioneerCam::imagePoint() const
{
  // ball in robot frame
  const auto rTb = wMr.getRotationMatrix().t() * (wTb - wMr.getTranslationVector());

  // robot -> cam
  vpHomogeneousMatrix cMr;
  const double c1 = cos(q[0]);
  const double c2 = cos(q[1]);
  const double s1 = sin(q[0]);
  const double s2 = sin(q[1]);
  cMr[0][0] = s1;
  cMr[0][1] = -c1;
  cMr[0][2] = 0;
  cMr[0][3] = -a1*s1;
  cMr[1][0] = -s2*c1;
  cMr[1][1] = -s1*s2;
  cMr[1][2] = -c2;
  cMr[1][3] = a1*s2*c1 + r1*c2;
  cMr[2][0] = c1*c2;
  cMr[2][1] = s1*c2;
  cMr[2][2] = -s2;
  cMr[2][3] = -a1*c1*c2 + r1*s2;
  cMr[3][0] = 0;
  cMr[3][1] = 0;
  cMr[3][2] = 0;
  cMr[3][3] = 1.;

  auto T = cMr.getRotationMatrix() * rTb + cMr.getTranslationVector();

  //std::cout << "T: " << T.t() << std::endl;
  if(T[2] < 0)
    T *= -1;
  if(T[2] < 1e-3)
    T[2] = 0.1;

  vpFeaturePoint p;
  p.set_xyZ(T[0]/T[2], T[1]/T[2], T[2]);
  //std::cout << "xy @ " << p.get_x() << " " << p.get_y() << std::endl;
  return p;
}

vpColVector PioneerCam::navCommand(double d, double lambdaV, double lambdaW) const
{
  // target in robot frame
  const auto T{wMr.getRotationMatrix().t() * (wTt - wMr.getTranslationVector())};

  vpColVector vw(2);
  vw[0] = lambdaV * (T[0]-d);
  vw[1] = lambdaW * toPi(atan2(T[1], T[0]));
  return vw;
}

void PioneerCam::jsCallback(const Model &msg)
{
  q.resize(2, false);

  const auto joints{msg.joint_size()};

  for(int idx = 0; idx < joints; ++idx)
  {
    auto& joint{msg.joint(idx)};
    for(uint i = 0; i < joint_names.size(); ++i)
    {
      if(joint.name() == joint_names[i])
      {
        q[i] = toPi(joint.axis1().position());
        continue;
      }
    }
  }
}

void PioneerCam::poseCallback(const Pose &msg)
{
  auto &frame{msg.name()};
  const auto T{toTranslation(msg.position())};
  auto &r{msg.orientation()};

  if(frame == "pioneer")
  {
    wMr.insert(T);
    wMr.insert(vpQuaternionVector(r.x(), r.y(), r.z(), r.w()));
  }
  else if(frame == "pioneer::camera")
  {
    wMc.insert(T);
    wMc.insert(vpQuaternionVector(r.x(), r.y(), r.z(), r.w()));
  }
  else if(frame == "target")
  {
    wTt = T;
  }
  else if(frame == "ball")
  {
    wTb = T;
  }
}

void PioneerCam::imageCallback(const Image &msg)
{
  static cv::Mat im_raw, im_proc;
  im_raw.create(msg.height(), msg.width(), CV_8UC3);
  std::copy(msg.data().begin(), msg.data().end(), im_raw.data);
  cv::cvtColor(im_raw, im_raw, cv::COLOR_BGR2RGB);
  //detector.process(im_raw, im_proc);
  cv::imshow("Pioneer", im_raw);
  cv::waitKey(1);
}
