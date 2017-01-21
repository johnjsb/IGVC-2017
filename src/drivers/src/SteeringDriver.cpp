/*
 * Created By: William Ou
 * Created On: September 22, 2016
 * Description: This node is responsible for passing received twist messages over
 *              serial to the arduino controlling the robot
 *
 */

#include <SteeringDriver.h>

// The constructor for MyClass
/*
 * Precondition: portName must be a valid and open serial port on this machine.
 */
SteeringDriver::SteeringDriver(int argc, char **argv, std::string node_name,
    std::string portName, SerialStreamBuf::BaudRateEnum baud_rate) :
    snowBot(portName, baud_rate) {

    ros::init(argc, argv, node_name);
    // Setup NodeHandles
    ros::NodeHandle nh;
    ros::NodeHandle public_nh("~");

    // Setup Subscriber(s)
    std::string topic_to_subscribe_to = "/final_decision/command";
    int refresh_rate = 10;
    command_subscriber = nh.subscribe(topic_to_subscribe_to, refresh_rate, &SteeringDriver::commandCallBack, this);

}

// The callback function for the subscriber (my_subscriber).
// This is called whenever a new message is received
/*
 * Postcondition: sends the complete twist_msg through serial, by first sending the linear
 *   x, y, z components in that order then the angular x, y, z components in that order.
 */
void SteeringDriver::commandCallBack(const geometry_msgs::Twist::ConstPtr& twist_msg) {

    snowBot.writeFloat((float) twist_msg->linear.x);
    snowBot.writeFloat((float) twist_msg->linear.y);
    snowBot.writeFloat((float) twist_msg->linear.z);
    
    snowBot.writeFloat((float) twist_msg->angular.x);
    snowBot.writeFloat((float) twist_msg->angular.y);
    snowBot.writeFloat((float) twist_msg->angular.z);

  try {
    // used for debugging, to retrieve the twist message sent.
    const int TWIST_SIZE = 6;
    for (int i = 0; i < TWIST_SIZE; i++) {
      std::string str = snowBot.readProtocol();
      float ff = 0;
      memcpy(&ff, str.c_str(), sizeof(float));
      if (i < 3) {
        std::cout << "linear: " << char('x' + i) << " " << ff << std::endl;
      }
      else {
        std::cout << "angular: " << char('x' + i - 3) << " " << ff << std::endl;
      }
    }
  } catch (const char * msg) {
    std::cerr << "Time out when reading..." << std::endl;
    return;
  }
}

