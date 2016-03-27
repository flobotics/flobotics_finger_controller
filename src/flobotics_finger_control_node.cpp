#include "ros/ros.h"
#include <std_msgs/Int8.h>
//#include "serial/serial.h"
#include <flobotics_finger_messages/flobotics_finger_force_adc_values.h>
#include <flobotics_finger_messages/flobotics_finger_force_limit_values.h>
#include <flobotics_finger_messages/flobotics_finger_control_values.h>
#include <flobotics_finger_messages/flobotics_finger_servo_control_values.h>

#include <flobotics_finger_control_node.h>

#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>

int val_windup = 450;
int val_wind   = 350;
int val_windstop = 400;  

/**
 * receives force values from sensors
 */
void flobotics_finger_force_callback(const std_msgs::Float32MultiArray& msg)
{
  //ROS_INFO("I heard adc0: [%d], adc1: [%d], adc2: [%d], adc3: [%d], adc4: [%d], adc5: [%d], adc6: [%d], adc7: [%d]", msg.adc0, msg.adc1, msg.adc2, msg.adc3, msg.adc4, msg.adc5, msg.adc6, msg.adc7);
  adc0 = (1100 / 5) * msg.data[0];
  adc1 = (1100 / 5) * msg.data[1];
  adc2 = (1100 / 5) * msg.data[2];
  adc3 = (1100 / 5) * msg.data[3];
  adc4 = (1100 / 5) * msg.data[4];
  adc5 = (1100 / 5) * msg.data[5];
  adc6 = (1100 / 5) * msg.data[6];
  adc7 = (1100 / 5) * msg.data[7];
}

/*
 * receives limit values
 */
void flobotics_finger_force_limit_values_callback(const flobotics_finger_messages::flobotics_finger_force_limit_values& msg)
{
  ROS_INFO("proximal min:[%d] max:[%d] hold:[%d]\n", msg.proximal_limit_min_1, msg.proximal_limit_max_1, msg.proximal_limit_hold_1 );
  proximal_limit_max_1 = msg.proximal_limit_max_1;
  proximal_limit_min_1 = msg.proximal_limit_min_1;
  proximal_limit_hold_1 = msg.proximal_limit_hold_1;

  proximal_limit_min_2 = msg.proximal_limit_min_2;
  proximal_limit_max_2 = msg.proximal_limit_max_2;
  proximal_limit_hold_2 = msg.proximal_limit_hold_2;

  proximal_1_limit_min_1 = msg.proximal_1_limit_min_1;
  proximal_1_limit_max_1 = msg.proximal_1_limit_max_1;
  proximal_1_limit_hold_1 = msg.proximal_1_limit_hold_1;

  proximal_1_limit_min_2 = msg.proximal_1_limit_min_2;
  proximal_1_limit_max_2 = msg.proximal_1_limit_max_2;
  proximal_1_limit_hold_2 = msg.proximal_1_limit_hold_2;

  intermediate_limit_min_1 = msg.intermediate_limit_min_1;
  intermediate_limit_max_1 = msg.intermediate_limit_max_1;
  intermediate_limit_hold_1 = msg.intermediate_limit_hold_1;

  intermediate_limit_min_2 = msg.intermediate_limit_min_2;
  intermediate_limit_max_2 = msg.intermediate_limit_max_2;
  intermediate_limit_hold_2 = msg.intermediate_limit_hold_2;

  distal_limit_min_1 = msg.distal_limit_min_1;
  distal_limit_max_1 = msg.distal_limit_max_1;
  distal_limit_hold_1 = msg.distal_limit_hold_1;

  distal_limit_min_2 = msg.distal_limit_min_2;
  distal_limit_max_2 = msg.distal_limit_max_2;
  distal_limit_hold_2 = msg.distal_limit_hold_2;

}

/*
 * receives control mode
 */
void flobotics_finger_control_callback(const flobotics_finger_messages::flobotics_finger_control_values& msg)
{
  ROS_INFO("finger control mode :[%d]\n", msg.mode);

  mode = msg.mode;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "flobotics_finger_control_node");

  ros::NodeHandle nh_sub_force;
  ros::NodeHandle nh_pub_to_servos;
  ros::NodeHandle nh_sub_to_finger_control;
  ros::NodeHandle nh_sub_to_force_limits;


  ros::Subscriber sub = nh_sub_force.subscribe("adc_pi_plus_sub", 1000, flobotics_finger_force_callback);
  ros::Subscriber subToControl = nh_sub_to_finger_control.subscribe("flobotics_finger_force_limit_values", 1000, flobotics_finger_force_limit_values_callback);
  ros::Subscriber subToLimits = nh_sub_to_force_limits.subscribe("flobotics_finger_control", 1000, flobotics_finger_control_callback);

  //servo_pub = nh_pub_to_servos.advertise<flobotics_finger_messages::flobotics_finger_servo_control_values>("pololu_usc01a", 1000);
  servo_pub = nh_pub_to_servos.advertise<std_msgs::Int16MultiArray>("servo_pwm_pi_sub", 100);

  ros::Rate loop_rate(20); //20 hertz



  mode=STOP;

  while(ros::ok()){

    if(mode == STOP)
    {
      ROS_INFO("-----Mode 0 - Stop Drawback");
      stop_drawback();
      mode=10; //idle;
    }
    else if(mode == IDLE)
    {
      ;
    }
    else if(mode == START_DRAWBACK)
    {
      ROS_INFO("-----Mode 1 - Start Drawback");
      start_drawback();
      mode = IDLE;
    }
    else if(mode==PROXIMAL_1_DRAWBACK)
    {
      ROS_INFO("-----Mode 2 - Start Proximal 1 Drawback");
      proximal_1_drawback();
    }
    else if(mode==PROXIMAL_2_DRAWBACK)
    {
      ROS_INFO("-----Mode 3 - Start Proximal 2 Drawback");
      proximal_2_drawback();
    }
    else if(mode==INTERMEDIATE_DRAWBACK)
    {
      ROS_INFO("-----Mode 4 - Start Intermediate Drawback");
      intermediate_drawback();
    }
    else if(mode==DISTAL_DRAWBACK)
    {
      ROS_INFO("-----Mode 5 - Start Distal Drawback");
      distal_drawback();
    }


    ros::spinOnce();

    loop_rate.sleep();
        ++count;
  }




  ros::spin();

  return 0;
}

void start_drawback(){

  std_msgs::Int16MultiArray msg;
  msg.data.clear();

  if(adc0 < proximal_limit_min_1)
    msg.data.push_back(val_wind); //do winding
  else if(adc0 > proximal_limit_max_1)
    msg.data.push_back(val_windup); //winding up
  else
    msg.data.push_back(val_windstop); //stand still

  if(adc1 < proximal_limit_min_2)
    msg.data.push_back(val_wind); //do winding
  else if(adc1 > proximal_limit_max_2)
    msg.data.push_back(val_windup); //winding up
  else
    msg.data.push_back(val_windstop); //stand still

  if(adc2 < proximal_1_limit_min_1)
    msg.data.push_back(val_wind); //do winding
  else if(adc2 > proximal_1_limit_max_1)
    msg.data.push_back(val_windup); //winding up
  else
    msg.data.push_back(val_windstop); //stand still

  if(adc3 < proximal_1_limit_min_2)
    msg.data.push_back(val_wind); //do winding
  else if(adc3 > proximal_1_limit_max_2)
    msg.data.push_back(val_windup); //winding up
  else
    msg.data.push_back(val_windstop); //stand still

  if(adc4 < intermediate_limit_min_1)
    msg.data.push_back(val_wind); //do winding
  else if(adc4 > intermediate_limit_max_1)
    msg.data.push_back(val_windup); //winding up
  else
    msg.data.push_back(val_windstop); //stand still


  if(adc5 < intermediate_limit_min_2)
    msg.data.push_back(val_wind); //do winding
  else if(adc5 > intermediate_limit_max_2)
    msg.data.push_back(val_windup); //winding up
  else
    msg.data.push_back(val_windstop); //stand still

  if(adc6 < distal_limit_min_1)
    msg.data.push_back(val_wind); //do winding
  else if(adc6 > distal_limit_max_1)
    msg.data.push_back(val_windup); //winding up
  else
    msg.data.push_back(val_windstop); //stand still

  if(adc7 < distal_limit_min_2)
    msg.data.push_back(val_wind); //do winding
  else if(adc7 > distal_limit_max_2)
    msg.data.push_back(val_windup); //winding up
  else
    msg.data.push_back(val_windstop); //stand still



  servo_pub.publish(msg);
}

void stop_drawback(){
	std_msgs::Int16MultiArray msg;
	msg.data.clear();

	msg.data.push_back(val_windstop);
	msg.data.push_back(val_windstop);
	msg.data.push_back(val_windstop);
	msg.data.push_back(val_windstop);
	msg.data.push_back(val_windstop);
	msg.data.push_back(val_windstop);
	msg.data.push_back(val_windstop);
	msg.data.push_back(val_windstop);

  servo_pub.publish(msg);

}

void distal_drawback(){

	std_msgs::Int16MultiArray msg;
	msg.data.clear();

  if(adc0 < proximal_limit_min_1)
    msg.data.push_back(val_wind); //do winding
  else if(adc0 > proximal_limit_max_1)
    msg.data.push_back(val_windup); //winding up
  else
    msg.data.push_back(val_windstop); //stand still

  if(adc1 < proximal_limit_min_2)
    msg.data.push_back(val_wind); //do winding
  else if(adc1 > proximal_limit_max_2)
    msg.data.push_back(val_windup); //winding up
  else
    msg.data.push_back(val_windstop); //stand still

  msg.data.push_back(val_windstop);
  msg.data.push_back(val_windstop);
  msg.data.push_back(val_windstop);
  msg.data.push_back(val_windstop);
  msg.data.push_back(val_windstop);
  msg.data.push_back(val_windstop);

  servo_pub.publish(msg);
}

void intermediate_drawback(){
  
  std_msgs::Int16MultiArray msg;
  msg.data.clear();

  msg.data.push_back(val_windstop);
  msg.data.push_back(val_windstop);

  if(adc2 < proximal_1_limit_min_1)
    msg.data.push_back(val_wind); //do winding
  else if(adc2 > proximal_1_limit_max_1)
    msg.data.push_back(val_windup); //winding up
  else
    msg.data.push_back(val_windstop); //stand still

  if(adc3 < proximal_1_limit_min_2)
    msg.data.push_back(val_wind); //do winding
  else if(adc3 > proximal_1_limit_max_2)
    msg.data.push_back(val_windup); //winding up
  else
    msg.data.push_back(val_windstop); //stand still

  msg.data.push_back(val_windstop);
  msg.data.push_back(val_windstop);
  msg.data.push_back(val_windstop);
  msg.data.push_back(val_windstop);

  servo_pub.publish(msg);
}

void proximal_2_drawback(){
  
  std_msgs::Int16MultiArray msg;
  msg.data.clear();

  msg.data.push_back(val_windstop);
  msg.data.push_back(val_windstop);
  msg.data.push_back(val_windstop);
  msg.data.push_back(val_windstop);

    if(adc4 < intermediate_limit_min_1)
      msg.data.push_back(val_wind); //do winding
    else if(adc4 > intermediate_limit_max_1)
      msg.data.push_back(val_windup); //winding up
    else
      msg.data.push_back(val_windstop); //stand still


    if(adc5 < intermediate_limit_min_2)
      msg.data.push_back(val_wind); //do winding
    else if(adc5 > intermediate_limit_max_2)
      msg.data.push_back(val_windup); //winding up
    else
      msg.data.push_back(val_windstop); //stand still

  msg.data.push_back(val_windstop);
  msg.data.push_back(val_windstop);


  servo_pub.publish(msg);
}

void proximal_1_drawback(){
  
  std_msgs::Int16MultiArray msg;
  msg.data.clear();

  msg.data.push_back(val_windstop);
  msg.data.push_back(val_windstop);
  msg.data.push_back(val_windstop);
  msg.data.push_back(val_windstop);
  msg.data.push_back(val_windstop);
  msg.data.push_back(val_windstop);

  if(adc6 < distal_limit_min_1)
    msg.data.push_back(val_wind); //do winding
  else if(adc6 > distal_limit_max_1)
    msg.data.push_back(val_windup); //winding up
  else
    msg.data.push_back(val_windstop); //stand still

  if(adc7 < distal_limit_min_2)
    msg.data.push_back(val_wind); //do winding
  else if(adc7 > distal_limit_max_2)
    msg.data.push_back(val_windup); //winding up
  else
    msg.data.push_back(val_windstop); //stand still

  servo_pub.publish(msg);
}
