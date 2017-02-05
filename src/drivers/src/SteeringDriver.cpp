/*
 * Created By: William Ou
 * Created On: September 22, 2016
 * Description: This node is responsible for passing received twist messages over
 *              serial to the arduino controlling the robot
 *
 */

#include <SteeringDriver.h>

#define DEBUG 0

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

  snowBot.sendTwistMsg(twist_msg);

#if DEBUG
  std::string message = snowBot.readString();
  if (!snowBot.read_error()) {
    const char * msgPtr = message.c_str();
    const int NUM_OF_FLOATS = 6; // 6 floats in a twist_msg.
    float floatArr[NUM_OF_FLOATS];
    for (int i = 0; i < NUM_OF_FLOATS; i++) {
      memcpy(&floatArr[i], msgPtr + sizeof(float) * i, sizeof(float));
    }
    for (int i = 0; i < NUM_OF_FLOATS / 2; i++) {
      std::cout << "linear " << char('x' + i) << ": " << floatArr[i] << " ";
    }
    for (int i = 3; i < NUM_OF_FLOATS; i++) {
      std::cout << "angular " << char('x' + i - 3) << ": " << floatArr[i] << " ";
    }
    std::cout << std::endl;
  }
  else {
    std::cout << "Reading error!" << std::endl;
  }
#endif
}

