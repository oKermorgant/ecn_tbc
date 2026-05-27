#include <tbc_pioneer_qp/pioneer_cam.h>
#include <csignal>

using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

#ifdef FOR_IGNITION_FORTRESS
constexpr auto gz_bin{"ign"};
constexpr auto gz_full{"ignition"};
#else
constexpr auto gz_bin{"gz"};
constexpr auto gz_msgs{"gz"};
#endif

inline void setModelPose(const std::string &model, const vpTranslationVector &t)
{
  std::stringstream ss;
  ss << gz_bin << " service -s /world/pioneer_world/set_pose "
        "--reqtype " << gz_msgs << ".msgs.Pose "
        "--reptype " << gz_msgs << ".msgs.Boolean --timeout 5000 --req '";
  ss << "name: \"" << model << "\"";
  ss << ", position: {";
  ss << "x: " << t[0] <<  ", y: " << t[1] <<  ", z: " << t[2] <<  "}";
  ss << "' > /dev/null";
  system(ss.str().c_str());
}

struct Watchdog
{
  tbc_pioneer_qp::PioneerCam robot{false};
  bool was_reset{false};

  static bool is_running(const std::string &process)
  {
    const auto cmd{"ps -A | grep " + process}; 
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd.c_str(), "r"), pclose);
    if (!pipe) {
      throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
      result += buffer.data();
    }
    std::cout << "poll: '" << result << "'" << std::endl;
    return result.find(process) != result.npos;
  }

  Watchdog()
  {
    //resetWorld();
  }

  void resetWorld()
  {
    std::cout << "Resetting world" << std::endl;

    setModelPose("pioneer", {-1,0,0.2});
    setModelPose("ball", {5, -2, 1});
    setModelPose("target", {5, 5, 0});

    // set q to 0
    while(robot.joints().frobeniusNorm() > 1e-3)
    {
      const auto q{robot.joints()};
      robot.sendCommand({0,0,-.1*q[0],-.1*q[1]});
      loop(100ms);
    }
    robot.sendCommand({0,0,0,0});
    was_reset = true;
  }

  void loop(std::chrono::milliseconds delay = 1s)
  {
    static auto last_sync{Clock::now()};
    std::this_thread::sleep_until(last_sync + delay);
    last_sync = Clock::now();
  }

  bool stalled()
  {
    std::cout << "Polling" << std::endl;
    if(is_running("pioneer"))
    {
      was_reset = false;
      return false;
    }
    return !was_reset;
  }
};


int main()
{
  Watchdog watchdog;
  while(true)
  {
    if(watchdog.stalled())
      watchdog.resetWorld();
    watchdog.loop();
  }
}