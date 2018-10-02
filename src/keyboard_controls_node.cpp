#include <stdio.h>
#include <termios.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define LEFT 1
#define RIGHT 2
#define FORWARD 4
#define BACKWARD 8

#define key_W 119
#define key_A 97
#define key_S 115
#define key_D 100

/*
 * Credits: https://github.com/sdipendra/ros-projects/blob/master/src/keyboard_non_blocking_input/src/keyboard_non_blocking_input_node.cpp#L25
 */
char getch()
{
	fd_set set;
	struct timeval timeout;
	int rv;
	char buff = 0;
	int len = 1;
	int filedesc = 0;
	FD_ZERO(&set);
	FD_SET(filedesc, &set);
	
	timeout.tv_sec = 0;
	timeout.tv_usec = 100;

	rv = select(filedesc + 1, &set, NULL, NULL, &timeout);

	struct termios old = {0};
	if (tcgetattr(filedesc, &old) < 0)
		ROS_ERROR("tcsetattr()");
	old.c_lflag &= ~ICANON;
	old.c_lflag &= ~ECHO;
	old.c_cc[VMIN] = 1;
	old.c_cc[VTIME] = 0;
	if (tcsetattr(filedesc, TCSANOW, &old) < 0)
		ROS_ERROR("tcsetattr ICANON");

	if(rv == -1)
		ROS_ERROR("select");
	else if(rv == 0){
		return buff;
	}else
		read(filedesc, &buff, len );
		while(getch()){}

	old.c_lflag |= ICANON;
	old.c_lflag |= ECHO;
	if (tcsetattr(filedesc, TCSADRAIN, &old) < 0)
		ROS_ERROR ("tcsetattr ~ICANON");
	return (buff);
}

/*
geometry_msgs/Vector3 linear
  float64 x <-- 2D
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z <-- 2D
*/
ros::Publisher motor_control_pub;
void sendControllInput(const char& controlCode){
    ROS_INFO("SIGNAL:: Forwards: %d, BACKWARDS: %d, LEFT: %d, RIGHT: %d", 
	(controlCode&FORWARD), (controlCode&BACKWARD), (controlCode&LEFT), (controlCode&RIGHT));

    float linearVel = 0.0;
    float angularVel = 0.0;

    if(controlCode&FORWARD){
	linearVel += 0.5;
    }
    if(controlCode&BACKWARD){
	linearVel -= 0.5;
    }
    if(controlCode&LEFT){
	angularVel += 3.14;
    }
    if(controlCode&RIGHT){
	angularVel -= 3.14;
    }

    geometry_msgs::Twist msg;
    msg.linear.x = linearVel;
    msg.angular.z = angularVel;
    motor_control_pub.publish(msg);
}

int oldKeyCode = 0;
int main(int argc, char **argv){
    ros::init(argc, argv, "rosie_keyboard_controls");

    ros::NodeHandle n;

    motor_control_pub = n.advertise<geometry_msgs::Twist>("/motor_controller/twist",1);

    ros::Rate loop_rate(5);

    while(ros::ok()){
	char controllerInput = 0;
	
	int newKeyCode = getch();	

	int outputKeyCode = 0;

	//Sorry about this...
	if(oldKeyCode != 0 && newKeyCode == 0){
	    outputKeyCode = oldKeyCode;
	}else{
	    outputKeyCode = newKeyCode;
	}

	ROS_INFO("Key pressed: %d", outputKeyCode);	

	if(outputKeyCode == key_W)
	    controllerInput = controllerInput|FORWARD;
	if(outputKeyCode == key_A)
	    controllerInput = controllerInput|LEFT;
	if(outputKeyCode == key_S)
	    controllerInput = controllerInput|BACKWARD;
	if(outputKeyCode == key_D)
	    controllerInput = controllerInput|RIGHT;

	sendControllInput(controllerInput);
        ros::spinOnce();
        loop_rate.sleep();

	oldKeyCode = newKeyCode;
    }
}
