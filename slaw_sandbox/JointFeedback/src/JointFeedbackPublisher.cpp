#include <youbot/YouBotBase.hpp>
#include <youbot/YouBotManipulator.hpp>
#include <ros/console.h>

using namespace youbot;

int main() {

	std::vector<JointSensedCurrent> armSensedCurrents;
	float armCurrents [5];

	try {
		
		// The youbot manipulator, combining all joints
	  YouBotManipulator* myYouBotManipulator = 0;

	  myYouBotManipulator = new YouBotManipulator("youbot-manipulator-test", "/home/youbot/ros/youbot_driver/config/");
	  myYouBotManipulator->doJointCommutation();
	  myYouBotManipulator->calibrateManipulator();
	  /*	  YouBotManipulator* youbotManipulator = new YouBotManipulator("youbot-manipulator2","/home/youbot/ros/youbot_driver/config/");
		youbotManipulator->getJointData(armSensedCurrents);

		int i = 4;
		ROS_INFO("Start");
		while (!armSensedCurrents.empty() || i < 0)
		{
			JointSensedCurrent sensedCurrent = armSensedCurrents.back();
    		armCurrents[i] = sensedCurrent.current.value();
    		i--;

    		armSensedCurrents.pop_back();

    		ROS_INFO("%i: %f", i, sensedCurrent.current.value());
		}
		delete youbotManipulator;*/
	} catch (std::exception& e) {
		std::cout << e.what() << std::endl;
	} catch (...) {
		std::cout << "unhandled exception" << std::endl;
	}

	return 0;
}
