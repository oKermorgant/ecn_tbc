#include <tbc_pioneer_qp/pioneer_cam.h>
#include <visp/vpQuadProg.h>

int main(int argc, char** argv)
{
  tbc_pioneer_qp::PioneerCam robot;

  vpColVector u(4);

  vpMatrix J1(4,4), J2(4,4);
  J1[0][0] =  J1[1][1] = 1;
  J1[2][2] = J1[3][3] = .001;

  // image visibility
  vpColVector xy_lim = 0.8*robot.getCamLimits();

  // error vectors
  vpColVector e1(4);

  // QP matrices
  vpMatrix C(8,4);
  vpColVector d(8);

  // wheel velocity
  vpMatrix Jw(4,4);
  vpColVector ew(4);ew = .99*robot.wheelMaxW();
  Jw[0][0] = Jw[1][0] = 1/robot.wheelRadius();
  Jw[0][1] = Jw[3][1] = -robot.wheelBaseLine()/robot.wheelRadius();
  Jw[1][1] = Jw[2][1] = robot.wheelBaseLine()/robot.wheelRadius();
  Jw[2][0] = Jw[3][0] = -1/robot.wheelRadius();

  // wheels
  C.insert(Jw, 0,0);
  d.insert(0, ew);

  // turning radius constraint
  vpMatrix A(1,4);
  vpColVector b(1);
  const double lc = 2;  // constraints gain

  // tilt limits +- 3*pi/4
  const auto tilt_lim{M_PI/4};
 // C[8][3] = 1;
 // C[9][3] = -1;

  vpQuadProg qp;

  while(robot.ok())
  {
    const auto vw{robot.navCommand(0.1, 0.5, 0.5)};
    A[0][1] = vw[0];
    A[0][0] = -vw[1];
    e1.insert(0, vw);
    e1.insert(2, -0.001*robot.joints());

    auto p{robot.imagePoint()};
    const auto s{p.get_s()};
    J2 = p.interaction() *robot.camJacobian();

    for(uint idx = 0; idx < 2; ++idx)
      std::cout << "|" << s[idx] << "| <= "<< xy_lim[idx] << std::endl;

    std::cout << "|" << robot.tilt() << "| <= " << tilt_lim << std::endl;


    C.insert(J2, 4, 0);
    d.insert(4, lc*(xy_lim - s));
    // im low
    C.insert(-J2, 6, 0);
    d.insert(6, lc*(xy_lim + s));

    // pan up
//    d[8] = lc*(tilt_lim - robot.tilt());
//    d[9] = lc*(tilt_lim + robot.tilt());

    qp.solveQP(J1, e1, A, b, C, d, u);
    //qp.solveQPe(J1, vw, A, b, u);

    //u.insert(0, vw);

    //u = -0.1 * J2.pseudoInverse() * s;

    std::cout << "Sending " << u.t() << std::endl;


    //u = -0.1 * J2.pseudoInverse() * s;
    robot.sendCommand(u);
  }
}
