#include <visp/vpFeaturePoint.h>
#include <tbc_baxter_vs/baxter_arm.h>
#include <ecn_common/butterworth.h>
#include <visp/vpSubMatrix.h>
#include <visp/vpSubColVector.h>


using namespace std;
using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  BaxterArm arm("right");    // defaults to right arm

  const auto period{20ms};
  const auto dt{period.count()/1000.0};

  // get joint limits
  const auto qmin{arm.jointMin()};
  const auto qmax{arm.jointMax()};

  // define a simple 2D point feature and its desired value
  vpFeaturePoint p,pd;
  pd.set_xyZ(0,0,1);

  // the error: ball (3) + horizon (1)
  vpColVector e(4);
  // desired area
  const auto area_d{arm.area_d()};

  // loop variables
  vpColVector qdot(7), s_prev;
  vpMatrix L(4, 6);
  L[2][2] = 1;

  // low-pass filter
  ecn_common::Butterworth_nD butter(2, 1, dt);

  auto control_loop = [&]()
  {
    cout << "-------------" << endl;

    // get point features
    const auto x{arm.x()};
    const auto y{arm.y()};
    const auto area{arm.area()};
    p.set_xyZ(x,y,1);
    std::cout << "x: " << x << ", y: " << y << ", area: " << area << '\n';

    // TODO update visual error vector e and corresponding interaction matrix

    // TODO update rotation part L to account for camera frame orientation x_c orthogonal to z_b
    const auto bRc = arm.cameraPose().getRotationMatrix();


    // TODO build H matrix (2nd section) using arm.rho()
    const auto q{arm.jointPosition()};

    // TODO compute feature Jacobian Js from L and arm.cameraJacobian()

    if(s_prev.size() && arm.use_feed_forward())
    {	  
      // TODO add feed-forward
      // sdot = Js.qdot + ds/dt
      // -> ds/dt = sdot - Js.qdot
      // edot = sdot = -lambda.e
      // -> Js.qdot = -lambda.e - ds/dt = -lambda.e - (sdot - Js.qdot)


    }
    s_prev = p.get_s();

    // TODO compute qdot, first without then with joint limits



    // send this command to the robot
    arm.setJointVelocity(qdot);

    // publish current VS error for rqt_plot
    arm.plot(e);
  };

  arm.useControl(control_loop, period);
}
