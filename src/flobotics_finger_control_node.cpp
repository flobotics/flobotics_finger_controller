#include "ros/ros.h"
#include <std_msgs/Int8.h>
#include "serial/serial.h"
#include <flobotics_finger_messages/flobotics_finger_force_adc_values.h>
#include <flobotics_finger_messages/flobotics_finger_force_limit_values.h>
#include <flobotics_finger_messages/flobotics_finger_control_values.h>
#include <flobotics_finger_messages/flobotics_finger_servo_control_values.h>

#include <flobotics_finger_control_node.h>



/**
 * receives force values from sensors
 */
void flobotics_finger_force_callback(const flobotics_finger_messages::flobotics_finger_force_adc_values& msg)
{
  //ROS_INFO("I heard adc0: [%d], adc1: [%d], adc2: [%d], adc3: [%d], adc4: [%d], adc5: [%d], adc6: [%d], adc7: [%d]", msg.adc0, msg.adc1, msg.adc2, msg.adc3, msg.adc4, msg.adc5, msg.adc6, msg.adc7);
  adc0 = msg.adc0;
  adc1 = msg.adc1;
  adc2 = msg.adc2;
  adc3 = msg.adc3;
  adc4 = msg.adc4;
  adc5 = msg.adc5;
  adc6 = msg.adc6;
  adc7 = msg.adc7;
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
  ROS_INFO("finger control bool:[%d]\n", msg.mode);

  mode = msg.mode;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "flobotics_finger_control_node");

  ros::NodeHandle nh_sub_force;
  ros::NodeHandle nh_pub_to_servos;
  ros::NodeHandle nh_sub_to_finger_control;
  ros::NodeHandle nh_sub_to_force_limits;


  ros::Subscriber sub = nh_sub_force.subscribe("adc", 1000, flobotics_finger_force_callback);
  ros::Subscriber subToControl = nh_sub_to_finger_control.subscribe("flobotics_finger_force_limit_values", 1000, flobotics_finger_force_limit_values_callback);
  ros::Subscriber subToLimits = nh_sub_to_force_limits.subscribe("flobotics_finger_control", 1000, flobotics_finger_control_callback);

  servo_pub = nh_pub_to_servos.advertise<flobotics_finger_messages::flobotics_finger_servo_control_values>("pololu_usc01a", 1000);

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

  if(adc0 < proximal_limit_min_1)
    msg.servo0 = 50; //do winding
  else if(adc0 > proximal_limit_max_1)
    msg.servo0 = 75; //winding up
  else
    msg.servo0 = 64; //stand still

  if(adc1 < proximal_limit_min_2)
    msg.servo1 = 50; //do winding
  else if(adc1 > proximal_limit_max_2)
    msg.servo1 = 75; //winding up
  else
    msg.servo1 = 64; //stand still

  if(adc2 < proximal_1_limit_min_1)
    msg.servo2 = 50; //do winding
  else if(adc2 > proximal_1_limit_max_1)
    msg.servo2 = 75; //winding up
  else
    msg.servo2 = 64; //stand still

  if(adc3 < proximal_1_limit_min_2)
    msg.servo3 = 50; //do winding
  else if(adc3 > proximal_1_limit_max_2)
    msg.servo3 = 75; //winding up
  else
    msg.servo3 = 64; //stand still

  if(adc4 < intermediate_limit_min_1)
    msg.servo4 = 50; //do winding
  else if(adc4 > intermediate_limit_max_1)
    msg.servo4 = 75; //winding up
  else
    msg.servo4 = 64; //stand still


  if(adc5 < intermediate_limit_min_2)
    msg.servo5 = 50; //do winding
  else if(adc5 > intermediate_limit_max_2)
    msg.servo5 = 75; //winding up
  else
    msg.servo5 = 64; //stand still

  if(adc6 < distal_limit_min_1)
    msg.servo6 = 50; //do winding
  else if(adc6 > distal_limit_max_1)
    msg.servo6 = 75; //winding up
  else
    msg.servo6 = 64; //stand still

  if(adc7 < distal_limit_min_2)
    msg.servo7 = 50; //do winding
  else if(adc7 > distal_limit_max_2)
    msg.servo7 = 75; //winding up
  else
    msg.servo7 = 64; //stand still

  msg.servo8 = 64;
  msg.servo9 = 64;
  msg.servo10 = 64;
  msg.servo11 = 64;
  msg.servo12 = 64;
  msg.servo13 = 64;
  msg.servo14 = 64;
  msg.servo15 = 64;




  if(cmp0 != msg.servo0 || cmp1 != msg.servo1 || cmp2 != msg.servo2 || cmp3 != msg.servo3 || cmp4 != msg.servo4 || cmp5 != msg.servo5 || cmp6 != msg.servo6 || cmp7 != msg.servo7){
    cmp0=msg.servo0;
    cmp1=msg.servo1;
    cmp2=msg.servo2;
    cmp3=msg.servo3;
    cmp4=msg.servo4;
    cmp5=msg.servo5;
    cmp6=msg.servo6;
    cmp7=msg.servo7;


    servo_pub.publish(msg);
  }
}

void stop_drawback(){
  msg.servo0 = 64;
  msg.servo1 = 64;
  msg.servo2 = 64;
  msg.servo3 = 64;
  msg.servo4 = 64;
  msg.servo5 = 64;
  msg.servo6 = 64;
  msg.servo7 = 64;
  msg.servo8 = 64;
  msg.servo9 = 64;
  msg.servo10 = 64;
  msg.servo11 = 64;
  msg.servo12 = 64;
  msg.servo13 = 64;
  msg.servo14 = 64;
  msg.servo15 = 64;

  servo_pub.publish(msg);

}

void distal_drawback(){
  if(adc0 < proximal_limit_min_1)
    msg.servo0 = 50; //do winding
  else if(adc0 > proximal_limit_max_1)
    msg.servo0 = 75; //winding up
  else
    msg.servo0 = 64; //stand still

  if(adc1 < proximal_limit_min_2)
    msg.servo1 = 50; //do winding
  else if(adc1 > proximal_limit_max_2)
    msg.servo1 = 75; //winding up
  else
    msg.servo1 = 64; //stand still

  msg.servo2 = 64;
  msg.servo3 = 64;
  msg.servo4 = 64;
  msg.servo5 = 64;
  msg.servo6 = 64;
  msg.servo7 = 64;
  msg.servo8 = 64;
  msg.servo9 = 64;
  msg.servo10 = 64;
  msg.servo11 = 64;
  msg.servo12 = 64;
  msg.servo13 = 64;
  msg.servo14 = 64;
  msg.servo15 = 64;

  servo_pub.publish(msg);
}

void intermediate_drawback(){
  if(adc2 < proximal_1_limit_min_1)
    msg.servo2 = 50; //do winding
  else if(adc2 > proximal_1_limit_max_1)
    msg.servo2 = 75; //winding up
  else
    msg.servo2 = 64; //stand still

  if(adc3 < proximal_1_limit_min_2)
    msg.servo3 = 50; //do winding
  else if(adc3 > proximal_1_limit_max_2)
    msg.servo3 = 75; //winding up
  else
    msg.servo3 = 64; //stand still

  msg.servo0 = 64;
  msg.servo1 = 64;
  msg.servo4 = 64;
  msg.servo5 = 64;
  msg.servo6 = 64;
  msg.servo7 = 64;
  msg.servo8 = 64;
  msg.servo9 = 64;
  msg.servo10 = 64;
  msg.servo11 = 64;
  msg.servo12 = 64;
  msg.servo13 = 64;
  msg.servo14 = 64;
  msg.servo15 = 64;

  servo_pub.publish(msg);
}

void proximal_2_drawback(){
  if(adc4 < intermediate_limit_min_1)
      msg.servo4 = 50; //do winding
    else if(adc4 > intermediate_limit_max_1)
      msg.servo4 = 75; //winding up
    else
      msg.servo4 = 64; //stand still


    if(adc5 < intermediate_limit_min_2)
      msg.servo5 = 50; //do winding
    else if(adc5 > intermediate_limit_max_2)
      msg.servo5 = 75; //winding up
    else
      msg.servo5 = 64; //stand still

    msg.servo0 = 64;
    msg.servo1 = 64;
    msg.servo2 = 64;
    msg.servo3 = 64;
    msg.servo6 = 64;
    msg.servo7 = 64;
    msg.servo8 = 64;
    msg.servo9 = 64;
    msg.servo10 = 64;
    msg.servo11 = 64;
    msg.servo12 = 64;
    msg.servo13 = 64;
    msg.servo14 = 64;
    msg.servo15 = 64;

    servo_pub.publish(msg);
}

void proximal_1_drawback(){
  if(adc6 < distal_limit_min_1)
    msg.servo6 = 50; //do winding
  else if(adc6 > distal_limit_max_1)
    msg.servo6 = 75; //winding up
  else
    msg.servo6 = 64; //stand still

  if(adc7 < distal_limit_min_2)
    msg.servo7 = 50; //do winding
  else if(adc7 > distal_limit_max_2)
    msg.servo7 = 75; //winding up
  else
    msg.servo7 = 64; //stand still

  msg.servo0 = 64;
  msg.servo1 = 64;
  msg.servo2 = 64;
  msg.servo3 = 64;
  msg.servo4 = 64;
  msg.servo5 = 64;
  msg.servo8 = 64;
  msg.servo9 = 64;
  msg.servo10 = 64;
  msg.servo11 = 64;
  msg.servo12 = 64;
  msg.servo13 = 64;
  msg.servo14 = 64;
  msg.servo15 = 64;

  servo_pub.publish(msg);
}
