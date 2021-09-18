/*********************************************************
 *     R O S - E S P C O P T E R    F I R M W A R E      *
 *                                                       *
 *     Created by Paulo Rezeck                           * 
 *     Computer Vision and Robotics Laboratory (VeRLab)  *
 *     December 12, 2019                                 *
 *                                                       *
 *     Adapted from original ESPCopter firware           *
 *********************************************************/



/*********************************************************
   INCLUDES
*********************************************************/
#include <ros.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <mavros_msgs/RCOut.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/BatteryStatus.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>
#include <ESP8266WiFi.h>
#include "espcopter.h"
/*********************************************************/


/*********************************************************
   COMPILATION OPTIONS
*********************************************************/
//#define ros_flag_attitude 1                 //subscribe to thrust/attitude commands (OLD)
//#define ros_flag_attitude_status 1        //publish drone's attitude
//#define ros_flag_battery 1                //publish battery status
//#define ros_flag_command_status 1         //publish command status???
#define ros_flag_imu 1                      //publish IMU data
//#define ros_flag_led 1                    //subscribe to RGB LED commands
//#define ros_flag_mag 1                    //publish magnetometer data
//#define ros_flag_motors 1                   //subscribe to mottors commands

#define ros_flag_mocap 1                    //subscribe to mocap data
//#define ros_flag_filter 1                   //publish filter data
#define ros_flag_acrorateref                //subscribe to acrorate refference
#define ros_flag_setgain                    //subscribe to topic to set the PID gains
/*********************************************************/


/*********************************************************
   WIFI SETUP
*********************************************************/
IPAddress ROS_MASTER_ADDRESS(192, 168, 2, 6); // ros master ip
//char* WIFI_SSID = "hero_network"; // network name
//char* WIFI_PASSWD = "s3cr3tp4ss"; // network password
char* WIFI_SSID = "corowap2G"; // network name
char* WIFI_PASSWD = "corowap2G"; // network password
/*********************************************************/



/*********************************************************
   ROS SETUP
*********************************************************/
String drone_name;
/* ROS Node Instaciatation */
ros::NodeHandle nh;

#ifdef ros_flag_led
/* LED callback */
void led_callback(const std_msgs::ColorRGBA& msg);
String led_topic;
ros::Subscriber<std_msgs::ColorRGBA> *led_sub;
#endif

bool enable_motors_only = false;
int16_t pwmMotorFL_, pwmMotorFR_, pwmMotorRL_, pwmMotorRR_;
#ifdef ros_flag_motors
/* Motor callback */
void motors_callback(const mavros_msgs::RCOut& msg);
String motors_topic;
ros::Subscriber<mavros_msgs::RCOut> *motors_sub;
#endif

#ifdef ros_flag_attitude
/* Attitude callback */
void attitude_callback(const mavros_msgs::AttitudeTarget& msg);
String attitude_topic;
ros::Subscriber<mavros_msgs::AttitudeTarget> *attitude_sub;
#endif

#ifdef ros_flag_mocap
/* Mocap pose callback*/
void mocap_callback(const geometry_msgs::PoseStamped& msg);
String mocap_topic;                 /* Topic name */
ros::Subscriber<geometry_msgs::PoseStamped> *mocap_sub;
#endif

#ifdef ros_flag_acrorateref
/* Acrorate reference callback*/
void acrorateref_callback(const geometry_msgs::Quaternion& msg);
String acrorateref_topic;                 /* Topic name */
ros::Subscriber<geometry_msgs::Quaternion> *acrorateref_sub;
#endif

#ifdef ros_flag_setgain
/* Acrorate reference callback*/
void setgain_callback(const geometry_msgs::Quaternion& msg);
String setgain_topic;                 /* Topic name */
ros::Subscriber<geometry_msgs::Point> *setgain_sub;
#endif

#ifdef ros_flag_battery
/* Battery Status */
mavros_msgs::BatteryStatus battery_msg;     /* Message Type */
String battery_topic;                 /* Topic name */
ros::Publisher *battery_pub;          /* Publisher */
void update_battery(void);            /* Update Loop */
#endif

#ifdef ros_flag_imu
/* IMU Status */
sensor_msgs::Imu imu_msg;     /* Message Type */
String imu_topic;                 /* Topic name */
ros::Publisher *imu_pub;          /* Publisher */
void update_imu(void);            /* Update Loop */
#endif

#ifdef ros_flag_mag
/* Magnetometer Status */
sensor_msgs::MagneticField mag_msg;     /* Message Type */
String mag_topic;                 /* Topic name */
ros::Publisher *mag_pub;          /* Publisher */
void update_mag(void);            /* Update Loop */
#endif

#ifdef ros_flag_command_status
/* Command Status */
mavros_msgs::AttitudeTarget command_msg;     /* Message Type */
String command_status_topic;                 /* Topic name */
ros::Publisher *command_pub;          /* Publisher */
void update_command(void);            /* Update Loop */
#endif

#ifdef ros_flag_attitude_status
/* Attitude Status */
mavros_msgs::AttitudeTarget attitude_msg;     /* Message Type */
String attitude_status_topic;                 /* Topic name */
ros::Publisher *attitude_pub;          /* Publisher */
void update_attitude(void);            /* Update Loop */
#endif


#ifdef ros_flag_filter
/* Publish filter states */
nav_msgs::Odometry filter_msg;     /* Message Type */
String filter_topic;                 /* Topic name */
ros::Publisher *filter_pub;          /* Publisher */
void update_filter(void);            /* Update Loop */
#endif



/* Arm motors service */
ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response> *arm_motors_srv;
String arm_motors_topic;
void arm_motors_callback(const std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

/* Gyro Calibration service */
ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response> *gyro_calibration_srv;
String gyro_calibration_topic;
double gyro_calibration_timer = -10000;
bool gyro_calibration_status = false;
int gyro_samples = 0;
void gyro_calibration_callback(const std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
void gyro_calibration_update();

/* Magnetometer Calibration service */
ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response> *mag_calibration_srv;
String mag_calibration_topic;
double mag_calibration_timer = -10000;
bool mag_calibration_status = false;
void mag_calibration_callback(const std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
void mag_calibration_update();
/*********************************************************/

char mbuf [200]; /* ROS Debug messages */

/* Timer */
double timer_ros, log_timer_ros, dt_ros = 20, timer_freq = 0, last_loop_freq = 0, loop_freq = 0, control_loop = 0;

/*********************************************************
   ARDUINO SETUP FUNCTION
*********************************************************/
void setup() {
  /* Connect the ESP8266 the the wifi AP */
  WiFi.begin(WIFI_SSID, WIFI_PASSWD);
  while (WiFi.status() != WL_CONNECTED) delay(500);

  Serial.begin(115200);
  Serial.println("[ROS] Setting up...");

  /* Start ROS communication module */
  uint16_t ROS_MASTER_PORT = 11411;
  nh.getHardware()->setConnection(ROS_MASTER_ADDRESS, ROS_MASTER_PORT);

  /* Setup laser publisher */
  drone_name = String("/drone_1");

#ifdef ros_flag_led
  led_topic = drone_name + String("/led");
  led_sub = new ros::Subscriber<std_msgs::ColorRGBA>(led_topic.c_str(), led_callback);
#endif

#ifdef ros_flag_motors
  motors_topic = drone_name + String("/motors");
  motors_sub = new ros::Subscriber<mavros_msgs::RCOut>(motors_topic.c_str(), motors_callback);
#endif

#ifdef ros_flag_attitude
  attitude_topic = drone_name + String("/attitude");
  attitude_sub = new ros::Subscriber<mavros_msgs::AttitudeTarget>(attitude_topic.c_str(), attitude_callback);
#endif

#ifdef ros_flag_mocap
  mocap_topic =  String("/mocap_node") + drone_name + String("/pose");
  mocap_sub = new ros::Subscriber<geometry_msgs::PoseStamped>(mocap_topic.c_str(), mocap_callback);
#endif

#ifdef ros_flag_acrorateref
  acrorateref_topic =  drone_name + String("/acrorate_ref");
  acrorateref_sub = new ros::Subscriber<geometry_msgs::Quaternion>(acrorateref_topic.c_str(), acrorateref_callback);
#endif

#ifdef ros_flag_setgain
  setgain_topic =  drone_name + String("/set_gain");
  setgain_sub = new ros::Subscriber<geometry_msgs::Point>(setgain_topic.c_str(), setgain_callback);
#endif

#ifdef ros_flag_battery
  battery_topic = drone_name + String("/battery");                         /* Update topic name */
  battery_pub = new ros::Publisher(battery_topic.c_str(), &battery_msg);    /* Instantiate publisher */
  nh.advertise(*battery_pub);
#endif

#ifdef ros_flag_imu
  imu_topic = drone_name + String("/imu");                         /* Update topic name */
  imu_pub = new ros::Publisher(imu_topic.c_str(), &imu_msg);    /* Instantiate publisher */
  nh.advertise(*imu_pub);
#endif

#ifdef ros_flag_mag
  mag_topic = drone_name + String("/mag");                         /* Update topic name */
  mag_pub = new ros::Publisher(mag_topic.c_str(), &mag_msg);    /* Instantiate publisher */
  nh.advertise(*mag_pub);
#endif

#ifdef ros_flag_attitude_status
  attitude_status_topic = drone_name + String("/attitude_status");                         /* Update topic name */
  attitude_pub = new ros::Publisher(attitude_status_topic.c_str(), &attitude_msg);    /* Instantiate publisher */
  nh.advertise(*attitude_pub);
#endif

#ifdef ros_flag_command_status
  command_status_topic = drone_name + String("/command_status");                         /* Update topic name */
  command_pub = new ros::Publisher(command_status_topic.c_str(), &command_msg);    /* Instantiate publisher */
  nh.advertise(*command_pub);
#endif

#ifdef ros_flag_filter
  filter_topic = drone_name + String("/filter");                         /* Update topic name */
  filter_pub = new ros::Publisher(filter_topic.c_str(), &filter_msg);    /* Instantiate publisher */
  filter_msg.header.frame_id = (drone_name + String("/base_link")).c_str(); /* Set frames only once */
  filter_msg.child_frame_id = String("world").c_str(); /* Set frames only once */
  nh.advertise(*filter_pub);
#endif

  arm_motors_topic = drone_name + String("/arm_motors");
  arm_motors_srv = new ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(arm_motors_topic.c_str(), &arm_motors_callback);
  nh.advertiseService(*arm_motors_srv);

  gyro_calibration_topic = drone_name + String("/gyro_calibration");
  gyro_calibration_srv = new ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(gyro_calibration_topic.c_str(), &gyro_calibration_callback);
  nh.advertiseService(*gyro_calibration_srv);

  mag_calibration_topic = drone_name + String("/mag_calibration");
  mag_calibration_srv = new ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response>(mag_calibration_topic.c_str(), &mag_calibration_callback);
  nh.advertiseService(*mag_calibration_srv);

  /* Starting ros node */
  nh.initNode();

  /* Address Subscribers */
#ifdef ros_flag_led
  nh.subscribe(*led_sub);
#endif
#ifdef ros_flag_motors
  nh.subscribe(*motors_sub);
#endif
#ifdef ros_flag_attitude
  nh.subscribe(*attitude_sub);
#endif
#ifdef ros_flag_mocap
  nh.subscribe(*mocap_sub);
#endif
#ifdef ros_flag_acrorateref
  nh.subscribe(*acrorateref_sub);
#endif
#ifdef ros_flag_setgain
  nh.subscribe(*setgain_sub);
#endif


  /* ROS LOG */
  sprintf(mbuf, "\33[96m[%s] R E A D Y !\33[0m", drone_name.c_str());
  nh.loginfo(mbuf);

  //Serial.println("[ROS] Ready!");

  mainSetup();
  //sprintf(mbuf, "\33[93m[%s][Calibration] Destination: [%f, %f], MagOffsetMotor: [%f, %f], MagOffset: [%f, %f, %f], MeanGyro: [%f, %f, %f]  \33[0m", drone_name.c_str(), ahrs.destination2[0], ahrs.destination2[1], ahrs.magOffsetMotorEpr[0], ahrs.magOffsetMotorEpr[1], ahrs.magOffset[0], ahrs.magOffset[1], ahrs.magOffset[2], ahrs.MEAN_GYRO[0], ahrs.MEAN_GYRO[1], ahrs.MEAN_GYRO[2]);
  //nh.loginfo(mbuf);
  sprintf(mbuf, "\33[93m[%s][Calibration] Destination: [%f, %f]\33[0m", drone_name.c_str(), ahrs.destination2[0], ahrs.destination2[1]);
  nh.loginfo(mbuf);
  sprintf(mbuf, "\33[93m[%s][Calibration] MagOffsetMotor: [%d, %d]\33[0m", drone_name.c_str(), ahrs.magOffsetMotorEpr[0], ahrs.magOffsetMotorEpr[1]);
  nh.loginfo(mbuf);
  sprintf(mbuf, "\33[93m[%s][Calibration] MagOffset: [%d, %d, %d]\33[0m", drone_name.c_str(), ahrs.magOffset[0], ahrs.magOffset[1], ahrs.magOffset[2]);
  nh.loginfo(mbuf);
  sprintf(mbuf, "\33[93m[%s][Calibration] MeanGyro: [%ld, %ld, %ld]\33[0m", drone_name.c_str(), ahrs.MEAN_GYRO[0], ahrs.MEAN_GYRO[1], ahrs.MEAN_GYRO[2]);
  nh.loginfo(mbuf);

  //Trim_Roll = -125; // -1750, 1750
  //Trim_Pitch = 250; // -1750, 1750
  //Trim_Yaw = 250;  // -1750, 1750

  delay(200);
}
/*********************************************************/



/*********************************************************
   ARDUINO LOOP FUNCTION
*********************************************************/
void loop() {

  /* ROS INFOS */
  if ((millis() - log_timer_ros) > 5000 && !gyro_calibration_status && !mag_calibration_status) {
    log_timer_ros = millis();
    sprintf(mbuf, "\33[96m[%s] Connected at time: %d, loop_freq: %f, battery: %f\33[0m", drone_name.c_str(), millis(), last_loop_freq, (float)analogRead(A0) * 5.5);
    nh.loginfo(mbuf);
    //sprintf(mbuf, "\33[96m[%s] Roll: %f, Pitch: %f, Yaw: %f\33[0m", drone_name.c_str(), roll.output, pitch.output, yaw.output);
    //nh.loginfo(mbuf);
    //sprintf(mbuf, "\33[96m[%s] throttle: %f, motorFL: %f, motorFR: %f, motorRL: %f, motorRR: %f\33[0m", drone_name.c_str(), throttle, motorFL, motorFR, motorRL, motorRR);
    //nh.loginfo(mbuf);
  }

  /* ROS Loop */
  if (micros() - timer_ros >= dt_ros*1000) {
    timer_ros = micros();

//    sprintf(mbuf, "\33[96m[%s] u_motors: %f, %f, %f, %f\33[0m", drone_name.c_str(), ahrs.u_motors[0], ahrs.u_motors[1], ahrs.u_motors[2], ahrs.u_motors[3]);
//    nh.loginfo(mbuf);

#ifdef ros_flag_battery
    update_battery();
#endif
#ifdef ros_flag_imu
    update_imu();
#endif
#ifdef ros_flag_mag
    update_mag();
#endif
#ifdef ros_flag_attitude_status
    update_attitude();
#endif
#ifdef ros_flag_command_status
    update_command();
#endif
#ifdef ros_flag_filter
    update_filter();
#endif
    gyro_calibration_update();
    mag_calibration_update();


//float st_tmp[9];
//ahrs.EKF_getStates(st_tmp);
//EKF_states
//sprintf(mbuf, "\33[96mpos: %d, %d, %d\nvel: %.2f, %.2f, %.2f\nang: %.2f, %.2f, %.2f\n\33[0m", st_tmp[0],st_tmp[1],st_tmp[2], st_tmp[3],st_tmp[4],st_tmp[5], st_tmp[6],st_tmp[7],st_tmp[8]);
//nh.loginfo(mbuf);

//Print ekf data
//sprintf(mbuf, "\33[96mpos: %.3f, %.3f, %.3f\nvel: %.3f, %.3f, %.3f\nang: %.3f, %.3f, %.3f\n\33[0m", ahrs.EKF_states[0],ahrs.EKF_states[1],ahrs.EKF_states[2], ahrs.EKF_states[3],ahrs.EKF_states[4],ahrs.EKF_states[5], ahrs.EKF_states[6],ahrs.EKF_states[7],ahrs.EKF_states[8]);
//nh.loginfo(mbuf);



    nh.spinOnce(); //[TODO] Call this more frequently?
  }



  
  if (enable_motors_only) {
    pwm_set_duty((pwmMotorFL_), 0);
    pwm_set_duty((pwmMotorFR_), 3);
    pwm_set_duty((pwmMotorRR_), 2);
    pwm_set_duty((pwmMotorRL_), 1);
    pwm_start();
  } else {
    if (!gyro_calibration_status && !mag_calibration_status) {
//      FlightControl();
      if(armControl){
        FlightControl_new();
      }
      else{
        pwm_set_duty((0), 0);
        pwm_set_duty((0), 3);
        pwm_set_duty((0), 2);
        pwm_set_duty((0), 1);
        pwm_start();
      }
    }
  }

  if (millis() - timer_freq < 1000) {
    loop_freq++;
  } else {
    last_loop_freq = loop_freq;
    loop_freq = 0;
    timer_freq = millis();
  }

}
/*********************************************************/


/*********************************************************
   LED CALLBACK FUNCTION
*********************************************************/
void led_callback(const std_msgs::ColorRGBA& msg) {
  analogWrite(redLed, (1.0-msg.r)*1024);
  analogWrite(greenLed, (1.0-msg.g)*1024);
  analogWrite(blueLed, (msg.b)*1024);
  sprintf(mbuf, "\33[96m[%s] Turning the leds on RGB(%d, %d, %d)...\33[0m", drone_name.c_str(), (int)msg.r, (int)msg.g, (int)msg.b);
  nh.loginfo(mbuf);
}
/*********************************************************/



/*********************************************************
   ARM MOTORS CALLBACK FUNCTION
*********************************************************/
void arm_motors_callback(const std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  armControl = !armControl;
  //Reset the integrator
  if (!armControl){
    ahrs.int_err_omega[0] = 0;
    ahrs.int_err_omega[1] = 0;
    ahrs.int_err_omega[2] = 0;
  }
  analogWriteFreq(20000);
  if (armControl == 1) {
    sprintf(mbuf, "\33[91m[%s] Be careful! Motors are enabled!\33[0m", drone_name.c_str());
    for (int i = 0; i < 50; i += 2) {
      pwm_set_duty((i), 0);
      pwm_set_duty((i), 3);
      pwm_set_duty((i), 2);
      pwm_set_duty((i), 1);
      pwm_start();
      delay(25);
    }
  } else {
    enable_motors_only = false;
    throttle = 0;
    sprintf(mbuf, "\33[92m[%s] Motors are disabled!\33[0m", drone_name.c_str());
  }
  analogWriteFreq(20000);
  nh.loginfo(mbuf);
}
/*********************************************************/


#ifdef ros_flag_motors
/*********************************************************
   RC MOTORS CALLBACK
*********************************************************/
void motors_callback(const mavros_msgs::RCOut& msg) {
  if (!armControl) {
    sprintf(mbuf, "\33[93m[%s] Motors are disarmed!\33[0m", drone_name.c_str());
    nh.loginfo(mbuf);
    enable_motors_only = false;
    return;
  }
  sprintf(mbuf, "\33[93m[%s] Settings motors to: %d, %d, %d, %d \33[0m", drone_name.c_str(), msg.channels[0], msg.channels[1], msg.channels[2], msg.channels[3]);
  nh.loginfo(mbuf);

  pwmMotorFL_ = round(map(msg.channels[0], 0, 1023, 0, PWM_PERIOD)); //255
  pwmMotorFR_ = round(map(msg.channels[3], 0, 1023, 0, PWM_PERIOD));
  pwmMotorRL_ = round(map(msg.channels[2], 0, 1023, 0, PWM_PERIOD));
  pwmMotorRR_ = round(map(msg.channels[1], 0, 1023, 0, PWM_PERIOD));

  pwmMotorFL_ = constrain(pwmMotorFL_, 0, PWM_PERIOD);
  pwmMotorFR_ = constrain(pwmMotorFR_, 0, PWM_PERIOD);
  pwmMotorRL_ = constrain(pwmMotorRL_, 0, PWM_PERIOD);
  pwmMotorRR_ = constrain(pwmMotorRR_, 0, PWM_PERIOD);
  enable_motors_only = true;
}
/*********************************************************/
#endif



#ifdef ros_flag_attitude
/*********************************************************
   ATTITUDE CONTROL CALLBACK
*********************************************************/
void attitude_callback(const mavros_msgs::AttitudeTarget& msg) {
  enable_motors_only = false;
  if (!armControl) {
    sprintf(mbuf, "\33[93m[%s] Motors are disarmed!\33[0m", drone_name.c_str());
    nh.loginfo(mbuf);
    return;
  }
  throttle = msg.thrust;
  if (throttle >= motorMax) {
    throttle = motorMax;
  } else if (throttle <= 0) {
    throttle = 0;
  }
  SetPoint[0] = map((int)msg.body_rate.x, -100, 100, -ROLL_LIMIT, ROLL_LIMIT);
  SetPoint[1] = map((int)msg.body_rate.y, -100, 100, -PITCH_LIMIT, PITCH_LIMIT);
  SetPoint[2] = map((int)msg.body_rate.z, -100, 100, -YAW_LIMIT, YAW_LIMIT);
}
/*********************************************************/
#endif


#ifdef ros_flag_mocap
/*********************************************************
   MOCAP CALLBACK
*********************************************************/

int mocap_count = 0;
int mocap_last_time = 0;
void mocap_callback(const geometry_msgs::PoseStamped& msg) {
  

//  sprintf(mbuf, "\33[93m[%s] Got pos: %.3f, %.3f, %.3f\33[0m", drone_name.c_str(), msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
//  nh.loginfo(mbuf);
//  sprintf(mbuf, "\33[93m[%s] Got quat: %.3f, %.3f, %.3f, %.3f\33[0m\n", drone_name.c_str(), msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);
//  nh.loginfo(mbuf);

  //Call EKF update
  float p[3] = {msg.pose.position.x, msg.pose.position.y, msg.pose.position.z};
  float q[4] = {msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z};
  ahrs.EKF_update(p,q);


//  //Position
//  filter_msg.pose.pose.position.x = msg.pose.position.x;
//  filter_msg.pose.pose.position.y = msg.pose.position.y;
//  filter_msg.pose.pose.position.z = msg.pose.position.z;
//
//  //Orientation
//  filter_msg.pose.pose.orientation.w = msg.pose.orientation.w;
//  filter_msg.pose.pose.orientation.x = msg.pose.orientation.x;
//  filter_msg.pose.pose.orientation.y = msg.pose.orientation.y;
//  filter_msg.pose.pose.orientation.z = msg.pose.orientation.z;
//
//  //Angular velocities
//  filter_msg.twist.twist.angular.x = ahrs.gyro[0];
//  filter_msg.twist.twist.angular.y = ahrs.gyro[1];
//  filter_msg.twist.twist.angular.z = ahrs.gyro[2];
//
//  filter_msg.header.frame_id = "/world"; /* Set frames only once */
//  filter_msg.child_frame_id = "/base_link";
//  
//  filter_msg.header.stamp = nh.now();
//  filter_pub->publish(& filter_msg);

  

//  mocap_count++;
//  if (millis()-mocap_last_time >= 1000){
//    double mocap_freq;
//
//    mocap_freq = 1000*mocap_count/float(millis()-mocap_last_time);
//    mocap_last_time = millis();
//    mocap_count = 0;
//    sprintf(mbuf, "\33[46m[%s] Mocap freq: %.4f\33[0m", drone_name.c_str(), mocap_freq);
//    nh.loginfo(mbuf);
//  }
}
/*********************************************************/
#endif





#ifdef ros_flag_acrorateref
/*********************************************************
   ACRORATE REF CALLBACK
*********************************************************/
void acrorateref_callback(const geometry_msgs::Quaternion& msg) {

  ahrs.acrorateref[0] = msg.w;
  ahrs.acrorateref[1] = msg.x;
  ahrs.acrorateref[2] = msg.y;
  ahrs.acrorateref[3] = msg.z;
}
/*********************************************************/
#endif

#ifdef ros_flag_setgain
/*********************************************************
   ACRORATE REF CALLBACK
*********************************************************/
void setgain_callback(const geometry_msgs::Point& msg) {

  

//  ahrs.Kpwx = msg.x;
//  ahrs.Kpwy = msg.y;
//  ahrs.Kpwz = msg.z;
//  sprintf(mbuf, "\33[93m[%s] Proportional gain set to: %.3f, %.3f, %.3f\33[0m\n", drone_name.c_str(), msg.x, msg.y, msg.z);
//  nh.loginfo(mbuf);

  ahrs.Kiwx = msg.x;
  ahrs.Kiwy = msg.y;
  ahrs.Kiwz = msg.z;
  sprintf(mbuf, "\33[93m[%s] Integral gain set to: %.3f, %.3f, %.3f\33[0m\n", drone_name.c_str(), msg.x, msg.y, msg.z);
  nh.loginfo(mbuf);

//  ahrs.Kdwx = msg.x;
//  ahrs.Kdwy = msg.y;
//  ahrs.Kdwz = msg.z;
//  sprintf(mbuf, "\33[93m[%s] Derivative gain set to: %.3f, %.3f, %.3f\33[0m\n", drone_name.c_str(), msg.x, msg.y, msg.z);
//  nh.loginfo(mbuf);
}
/*********************************************************/
#endif



#ifdef ros_flag_command_status
/*********************************************************
   COMMAND STATUS PUBLISHER
*********************************************************/
void update_command(void) {
  command_msg.body_rate.x = roll.output;
  command_msg.body_rate.y = pitch.output;
  command_msg.body_rate.z = yaw.output;
  command_msg.thrust = throttle;
  command_msg.header.stamp = nh.now();
  command_pub->publish(& command_msg);
}
/*********************************************************/
#endif



#ifdef ros_flag_mag
/*********************************************************
   MAG STATUS PUBLISHER
*********************************************************/
void update_mag(void) {
  mag_msg.magnetic_field.x = ahrs.mag[0];
  mag_msg.magnetic_field.y = ahrs.mag[1];
  mag_msg.magnetic_field.z = ahrs.mag[2];
  
  mag_msg.header.stamp = nh.now();
  mag_pub->publish(& mag_msg);
}
/*********************************************************/
#endif



#ifdef ros_flag_imu
/*********************************************************
   IMU STATUS PUBLISHER
*********************************************************/
void update_imu(void) {
  imu_msg.angular_velocity.x = ahrs.gyro[0];
  imu_msg.angular_velocity.y = ahrs.gyro[1];
  imu_msg.angular_velocity.z = ahrs.gyro[2];

//  imu_msg.linear_acceleration.x = ahrs.accel_angle[0];
//  imu_msg.linear_acceleration.y = ahrs.accel_angle[1];
//  imu_msg.linear_acceleration.z = ahrs.accel_angle[2];
  imu_msg.linear_acceleration.x = ahrs.accel_si[0]; //[Adriano]
  imu_msg.linear_acceleration.y = ahrs.accel_si[1]; //[Adriano]
  imu_msg.linear_acceleration.z = ahrs.accel_si[2]; //[Adriano]

imu_msg.orientation.w = ahrs.EKF_states[6]; //[Adriano]
imu_msg.orientation.x = ahrs.EKF_states[7]; //[Adriano]
imu_msg.orientation.y = ahrs.EKF_states[8]; //[Adriano]
imu_msg.orientation.z = ahrs.EKF_states[9]; //[Adriano]
  
  imu_msg.header.stamp = nh.now();
  imu_pub->publish(& imu_msg);
}
/*********************************************************/
#endif



#ifdef ros_flag_attitude_status
/*********************************************************
   ATTITUDE STATUS PUBLISHER
*********************************************************/
void update_attitude(void) {
  attitude_msg.orientation.x = attitude[1];
  attitude_msg.orientation.y = attitude[0];
  attitude_msg.orientation.z = attitude[2];
  attitude_msg.orientation.w = degree[0];
  attitude_msg.body_rate.x = rate[1];
  attitude_msg.body_rate.y = rate[0];
  attitude_msg.body_rate.z = rate[2];
  attitude_msg.thrust = throttle;
  attitude_msg.header.stamp = nh.now();
  attitude_pub->publish(& attitude_msg);

}
/*********************************************************/
#endif


#ifdef ros_flag_battery
/*********************************************************
   BATTERY STATUS PUBLISHER
*********************************************************/
void update_battery(void) {
  battery_msg.voltage = analogRead(A0) * 5.5;
  battery_msg.remaining = battery_msg.voltage - 3000;
  battery_msg.header.stamp = nh.now();
  battery_pub->publish( &battery_msg );
}
/*********************************************************/
#endif

#ifdef ros_flag_filter
/*********************************************************
   FILTER STATUS PUBLISHER
*********************************************************/
void update_filter(void) {
//  filter_msg.magnetic_field.x = ahrs.mag[0];
//  filter_msg.magnetic_field.y = ahrs.mag[1];
//  filter_msg.magnetic_field.z = ahrs.mag[2];

  //Position
  filter_msg.pose.pose.position.x = ahrs.EKF_states[0];
  filter_msg.pose.pose.position.y = ahrs.EKF_states[1];
  filter_msg.pose.pose.position.z = ahrs.EKF_states[2];

  //Orientation
  filter_msg.pose.pose.orientation.w = ahrs.EKF_states[6];
  filter_msg.pose.pose.orientation.x = ahrs.EKF_states[7];
  filter_msg.pose.pose.orientation.y = ahrs.EKF_states[8];
  filter_msg.pose.pose.orientation.z = ahrs.EKF_states[9];

  //Linear velocities
  filter_msg.twist.twist.linear.x = ahrs.EKF_states[3];
  filter_msg.twist.twist.linear.y = ahrs.EKF_states[4];
  filter_msg.twist.twist.linear.z = ahrs.EKF_states[5];

  //Angular velocities
  filter_msg.twist.twist.angular.x = ahrs.gyro[0];
  filter_msg.twist.twist.angular.y = ahrs.gyro[1];
  filter_msg.twist.twist.angular.z = ahrs.gyro[2];

//  filter_msg.header.frame_id = (drone_name + String("/base_link")).c_str(); /* Set frames only once */
//  filter_msg.child_frame_id = (String("/world")).c_str(); /* Set frames only once */
//  filter_msg.header.frame_id = "/base_link"; /* Set frames only once */
//  filter_msg.child_frame_id = "/world";

  filter_msg.header.frame_id = "/world"; /* Set frames only once */
  filter_msg.child_frame_id = "/base_link";
  
  filter_msg.header.stamp = nh.now();
  filter_pub->publish(& filter_msg);
}
/*********************************************************/
#endif



/*********************************************************
   GYRO CALIBRATION SERVICE
*********************************************************/
void gyro_calibration_callback(const std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  sprintf(mbuf, "\33[93m[%s] Starting gyro calibration...\33[0m", drone_name.c_str());
  nh.loginfo(mbuf);
  gyro_calibration_timer = millis();
  gyro_calibration_status = true;
  gyro_samples = 0;
}

void gyro_calibration_update() {
  if (!gyro_calibration_status) return;

  double tnow = millis() - gyro_calibration_timer;
  static bool print_once_gyro = true;

  if (tnow < 5000) {
    if (print_once_gyro) {
      sprintf(mbuf, "\33[91m[%s] DO NOT TOUCH THE DRONE!\33[0m", drone_name.c_str());
      nh.loginfo(mbuf);
      print_once_gyro = false;
    }
  } else if (tnow > 5000 && tnow < 10000) {
    ahrs.I2Cread(MPU9250_ADDRESS, 0x3B, 14, ahrs.Buf);
    ahrs.gyro_int[0] = (ahrs.Buf[8] << 8 | ahrs.Buf[9]);
    ahrs.gyro_int[1] = (ahrs.Buf[10] << 8 | ahrs.Buf[11]);
    ahrs.gyro_int[2] = ahrs.Buf[12] << 8 | ahrs.Buf[13];
    ahrs.SUM_GYRO[0] += ahrs.gyro_int[0];
    ahrs.SUM_GYRO[1] += ahrs.gyro_int[1];
    ahrs.SUM_GYRO[2] += ahrs.gyro_int[2];
    gyro_samples++;
  } else if (tnow > 10000 && tnow < 10500) {
    EEPROM.begin(512);
    for (int i = 0; i < 3; i++) {
      ahrs.MEAN_GYRO[i] = ahrs.SUM_GYRO[i] / gyro_samples;
      if (ahrs.MEAN_GYRO[i] < 0) {
        EEPROM.write(4 + i, 1);
        EEPROM.write(1 + i, -(int8_t)(ahrs.MEAN_GYRO[i]));
      } else {
        EEPROM.write(4 + i, 0);
        EEPROM.write(1 + i, (int8_t)(ahrs.MEAN_GYRO[i]));
      }
    }
    EEPROM.end();

    sprintf(mbuf, "\33[92m[%s] Gyro is calibrated!\33[0m", drone_name.c_str());
    nh.loginfo(mbuf);
    sprintf(mbuf, "\33[92m[%s] Mean Gyro: %ld %ld %ld\33[0m", drone_name.c_str(), ahrs.MEAN_GYRO[0], ahrs.MEAN_GYRO[1], ahrs.MEAN_GYRO[2]);
    nh.loginfo(mbuf);
    gyro_calibration_status = false;
    print_once_gyro = true;
  }

}
/*********************************************************/




/*********************************************************
   MAGNETOMETER CALIBRATION SERVICE
*********************************************************/
void mag_calibration_callback(const std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  sprintf(mbuf, "\33[93m[%s] Starting magnetometer calibration...\33[0m", drone_name.c_str());
  nh.loginfo(mbuf);
  mag_calibration_timer = millis();
  mag_calibration_status = true;
}

void mag_calibration_update() {
  if (!mag_calibration_status) return;

  double tnow = millis() - mag_calibration_timer;

  static float attitudeFake[3] = {0};
  static float rateFake[3] = {0};
  static float degreeFake[3] = {0};
  static float degreeFake_[3] = {0};
  static float magOffsetMotorFnlFake[3] = {0};
  static float minX, maxX, minY, maxY;
  static float magOffsetMotorCount, magOffsetMotorCount2;
  static int16_t magOffset[3] = {0};
  static int16_t mag_scale[3] = {0};
  static float magOffsetMotor[3] = {0};

  static float magOffsetMotor2[3] = {0};
  static float sumMagOffsetMotor2[3] = {0};
  static int16_t magOffsetMotorFnl[3] = {0};
  static int16_t magOffsetMotorFnl2[3] = {0};
  static int16_t magOffsetMotorEpr[2] = {0};
  static float destination2[3] = {1, 1, 1};

  static bool print_once_mag = true;
  static bool print_once_mag_l1 = true;
  static bool print_once_mag_l2 = true;

  if (tnow < 500) {
    print_once_mag = true;
    print_once_mag_l1 = true;
    print_once_mag_l2 = true;
    magOffsetMotorCount = 0.0;
    magOffsetMotorCount2 = 0.0;
    magOffsetMotor[0] = 0.0;
    magOffsetMotor[1] = 0.0;
    magOffsetMotor[2] = 0.0;
  } else if (tnow < 3000) {
    if (print_once_mag) {
      print_once_mag = false;
      magOffset[0] = 0;
      magOffset[1] = 0;
      magOffset[2] = 0;
      magOffsetMotorFnl[0] = 0;
      magOffsetMotorFnl[1] = 0;
      ahrs.headingMag(rateFake, attitudeFake, degreeFake , 0.0);
      minX = degreeFake[1];
      maxX = degreeFake[1];
      minY = degreeFake[2];
      maxY = degreeFake[2];
      sprintf(mbuf, "\33[91m[%s] TURN THE DRONE THREE TIMES C.W.!\33[0m", drone_name.c_str());
      nh.loginfo(mbuf);
    }
  } else if (tnow > 3000 && tnow < 15000) {
    ahrs.I2Cread(MPU9250_ADDRESS, 0x3B, 14, ahrs.Buf);
    ahrs.headingMag(rateFake, attitudeFake, degreeFake , 0);
    if (degreeFake[1] < minX) minX = degreeFake[1];
    if (degreeFake[1] > maxX) maxX = degreeFake[1];
    if (degreeFake[2] < minY) minY = degreeFake[2];
    if (degreeFake[2] > maxY) maxY = degreeFake[2];
    sprintf(mbuf, "\33[93m[%s] MagX: (%f, %f) MagY: (%f, %f) DegreeXY: (%f %f) \33[0m", drone_name.c_str(), minX, maxX, minY, maxY, degreeFake[1], degreeFake[2]);
    nh.loginfo(mbuf);
    print_once_mag = true;
  } else if (tnow > 15000 && tnow < 30000) {
    if (print_once_mag) {
      magOffset[0] = (int16_t)(maxX + minX) / 2.0;
      magOffset[1] = (int16_t)(maxY + minY) / 2.0;

      mag_scale[0]  = (int16_t)(maxX - minX) / 2.0; // get average x axis max chord length in counts
      mag_scale[1]  = (int16_t)(maxY - minY) / 2.0; // get average y axis max chord length in counts

      float avg_rad  = (float)(mag_scale[0] + mag_scale[1]) / 2.0;

      destination2[0] = avg_rad / ((float)mag_scale[0]);
      destination2[1] = avg_rad / ((float)mag_scale[1]);

      sprintf(mbuf, "\33[92m[%s] Mag Offset: %d, %d, %d\33[0m", drone_name.c_str(), magOffset[0], magOffset[1], magOffset[2]);
      nh.loginfo(mbuf);

      sprintf(mbuf, "\33[91m[%s] Stop Moving! Waiting for the rest of the process.\33[0m", drone_name.c_str());
      nh.loginfo(mbuf);
      print_once_mag = false;
    }
    ahrs.headingMag(rateFake, attitudeFake, degreeFake, 0 );
    if (degreeFake_[2] != degreeFake[2] && degreeFake_[1] != degreeFake[1] ) {
      magOffsetMotorCount += 1.0;
      magOffsetMotor[0] += degreeFake[1];
      magOffsetMotor[1] += degreeFake[2];
      degreeFake_[1] = degreeFake[1];
      degreeFake_[2] = degreeFake[2];
    }
  } else if (tnow > 30000 && tnow < 33000) {
    if (print_once_mag_l1) {
      sumMagOffsetMotor[0] = magOffsetMotor[0] / magOffsetMotorCount;
      sumMagOffsetMotor[1] = magOffsetMotor[1] / magOffsetMotorCount;
      sprintf(mbuf, "\33[93m[%s] SumMag: (%f, %f) with %d samples.\33[0m", drone_name.c_str(), sumMagOffsetMotor[0], sumMagOffsetMotor[1], (int)magOffsetMotorCount);
      nh.loginfo(mbuf);
      if (magOffsetMotorCount < 190.0) {
        sprintf(mbuf, "\33[91m[%s] Too few samples (Only %d)! Calibrationg failed!\33[0m", drone_name.c_str(), (int)magOffsetMotorCount);
        nh.loginfo(mbuf);
        mag_calibration_status = false;
        return;
      } else {
        sprintf(mbuf, "\33[92m[%s] Be careful! Checking the mag with the motors on.\33[0m", drone_name.c_str());
        nh.loginfo(mbuf);
      }
      print_once_mag_l1 = false;
    }
  } else if (tnow > 33000 && tnow < 33800) {
    if (print_once_mag_l2) {
      analogWriteFreq(20000);
      for (int i = 0; i < 75; i++) {
        pwm_set_duty((i), 0);
        pwm_set_duty((i), 3);
        pwm_set_duty((i), 2);
        pwm_set_duty((i), 1);
        pwm_start();
        delay(5);
      }
      print_once_mag_l2 = false;
    }
    pwm_set_duty((200), 0);
    pwm_set_duty((200), 3);
    pwm_set_duty((200), 2);
    pwm_set_duty((200), 1);
    pwm_start();
    ahrs.headingMag(rateFake, attitudeFake, degreeFake, 0.0);
    if (degreeFake_[1] != degreeFake[1] && degreeFake_[2] != degreeFake[2] ) {
      magOffsetMotorCount2 += 1.0;
      degreeFake_[1] = degreeFake[1]; // try with the sum
      degreeFake_[2] = degreeFake[2];
    }
  } else if (tnow > 33800) {
    for (int i = 0; i < 10; i++) {
      pwm_set_duty((0), 0);
      pwm_set_duty((0), 3);
      pwm_set_duty((0), 2);
      pwm_set_duty((0), 1);
      pwm_start();
      delay(25);
    }

    sumMagOffsetMotor2[0] = degreeFake[1];
    sumMagOffsetMotor2[1] = degreeFake[2];
    sprintf(mbuf, "\33[93m[%s] SumMag2: (%f, %f) with %d samples.\33[0m", drone_name.c_str(), sumMagOffsetMotor2[0], sumMagOffsetMotor2[1], (int)magOffsetMotorCount2);
    nh.loginfo(mbuf);
    if (magOffsetMotorCount2 < 5) {
      sprintf(mbuf, "\33[91m[%s] Too few samples (Only %d)! Calibrationg failed!\33[0m", drone_name.c_str(), (int)magOffsetMotorCount2);
      nh.loginfo(mbuf);
      mag_calibration_status = false;
      return;
    }

    magOffsetMotorEpr[0] = (int16_t)(sumMagOffsetMotor[0] - sumMagOffsetMotor2[0]);
    magOffsetMotorEpr[1] = (int16_t)(sumMagOffsetMotor[1] - sumMagOffsetMotor2[1]);

    sprintf(mbuf, "\33[93m[%s] MagEpr: (%d, %d)\33[0m", drone_name.c_str(), magOffsetMotorEpr[0], magOffsetMotorEpr[1]);
    nh.loginfo(mbuf);

    ahrs.destination2[0] = destination2[0];
    ahrs.destination2[1] = destination2[1];

    ahrs.magOffset[0] = magOffset[0];
    ahrs.magOffset[1] = magOffset[1];
    ahrs.magOffset[2] = magOffset[2];

    ahrs.magOffsetMotorEpr[0] = magOffsetMotorEpr[0];
    ahrs.magOffsetMotorEpr[1] = magOffsetMotorEpr[1];

    EEPROM.begin(512);

    EEPROM.write(12, highByte(int16_t(destination2[0] * 100)));
    EEPROM.write(13, lowByte(int16_t(destination2[0] * 100)));

    EEPROM.write(14, highByte(int16_t(destination2[1] * 100)));
    EEPROM.write(15, lowByte(int16_t(destination2[1] * 100)));

    EEPROM.write(10, highByte(magOffset[0]));
    EEPROM.write(11, lowByte(magOffset[0]));

    EEPROM.write(20, highByte(magOffset[1]));
    EEPROM.write(21, lowByte(magOffset[1]));

    EEPROM.write(30, highByte(magOffset[2]));
    EEPROM.write(31, lowByte(magOffset[2]));

    EEPROM.write(40, highByte(magOffsetMotorEpr[0]));
    EEPROM.write(41, lowByte(magOffsetMotorEpr[0]));

    EEPROM.write(50, highByte(magOffsetMotorEpr[1]));
    EEPROM.write(51, lowByte(magOffsetMotorEpr[1]));

    EEPROM.write(35, highByte(0));
    EEPROM.write(36, lowByte(0));

    EEPROM.write(37, highByte(0));
    EEPROM.write(38, lowByte(0));

    EEPROM.write(42, highByte(0));
    EEPROM.write(43, lowByte(0));

    EEPROM.end();

    sprintf(mbuf, "\33[92m[%s] Magnetometer is calibrated!\33[0m", drone_name.c_str());
    nh.loginfo(mbuf);
    sprintf(mbuf, "\33[92m[%s] Destination: (%f, %f)\33[0m", drone_name.c_str(), destination2[0], destination2[1]);
    nh.loginfo(mbuf);
    sprintf(mbuf, "\33[92m[%s] magOffset: (%d, %d, %d)\33[0m", drone_name.c_str(), magOffset[0], magOffset[1], magOffset[2]);
    nh.loginfo(mbuf);
    sprintf(mbuf, "\33[92m[%s] magOffsetMotorEpr: (%d, %d)\33[0m", drone_name.c_str(), magOffsetMotorEpr[0], magOffsetMotorEpr[1]);
    nh.loginfo(mbuf);
    mag_calibration_status = false;
  }

}
/*********************************************************/


/*********************************************************
   E N D
*********************************************************/
