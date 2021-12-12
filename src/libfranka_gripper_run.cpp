// Only run this code of franka_ros/franka_gripper is not running in the background!! - Nadia
// From: https://github.com/nbfigueroa/franka_gripper_run

#include <iostream>
#include <sstream>
#include <string>

#include <franka/exception.h>
#include <franka/gripper.h>

int main(int argc, char** argv) {

  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <gripper-hostname> <open/close>" << std::endl;
    return -1;
  }
  try {
    std::string franka_ip = "172.16.0.2";
    franka::Gripper gripper(franka_ip);
    std::stringstream ss(argv[1]);
    bool open_close;
    if (!(ss >> open_close)) {
      std::cerr << "<open/close> must be either 0 (close) or 1 (open)." << std::endl;
      return -1;
    }

    if (open_close) {
      // open
      bool success = gripper.move(0.08, 0.1);

      std::cout << "Opening gripper" << std::endl;
      std::cout << "Success: " << success << std::endl;
    } else {
      // close
      bool success = gripper.grasp(0, 0.1, 30, 0.2, 0.2);

      std::cout << "Closing gripper" << std::endl;
      std::cout << "Success: " << success << std::endl;
    }

    franka::GripperState state = gripper.readOnce();

    std::cout << "Gripper state: " << state << std::endl;

  } catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;

}