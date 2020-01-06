/*********************************************************
 *    R O S - E S P C O P T E R    F I R M W A R E       * 
 *                                                       *
 *    Created by Paulo Rezeck                            *
 *    Computer Vision and Robotics Laboratory (VeRLab)   *
 *    December 12, 2019                                  *
*********************************************************/



/*********************************************************
 * INCLUDES
*********************************************************/
#include <ros.h>
#include <std_msgs/ColorRGBA.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <mavros_msgs/RCOut.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/BatteryStatus.h>
#include <std_srvs/Trigger.h>
#include <ESP8266WiFi.h>
#include "espcopter.h"
/*********************************************************/



/*********************************************************
 * WIFI SETUP
*********************************************************/
IPAddress ROS_MASTER_ADDRESS(10, 42, 0, 1); // ros master ip
char* WIFI_SSID = "hero_network"; // network name
char* WIFI_PASSWD = "s3cr3tp4ss"; // network password
/*********************************************************/



/*********************************************************
 * ROS SETUP
*********************************************************/
String drone_name;
/* ROS Node Instaciatation */
ros::NodeHandle nh;

/* LED callback */
void led_callback(const std_msgs::ColorRGBA& msg);
String led_topic;
ros::Subscriber<std_msgs::ColorRGBA> *led_sub;

/* Motor callback */
bool enable_motors_only = false;
int16_t pwmMotorFL_, pwmMotorFR_, pwmMotorRL_, pwmMotorRR_;
void motors_callback(const mavros_msgs::RCOut& msg);
String motors_topic;
ros::Subscriber<mavros_msgs::RCOut> *motors_sub;

/* Attitude callback */
void attitude_callback(const mavros_msgs::AttitudeTarget& msg);
String attitude_topic;
ros::Subscriber<mavros_msgs::AttitudeTarget> *attitude_sub;

/* Battery Status */
mavros_msgs::BatteryStatus battery_msg;     /* Message Type */
String battery_topic;                 /* Topic name */
ros::Publisher *battery_pub;          /* Publisher */
void update_battery(void);            /* Update Loop */

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
double timer_ros, log_timer_ros, rate_ros = 50, timer_freq = 0, loop_freq = 0, control_loop = 0;

/*********************************************************
 * ARDUINO SETUP FUNCTION
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

  led_topic = drone_name + String("/led");
  led_sub = new ros::Subscriber<std_msgs::ColorRGBA>(led_topic.c_str(), led_callback);

  motors_topic = drone_name + String("/motors");
  motors_sub = new ros::Subscriber<mavros_msgs::RCOut>(motors_topic.c_str(), motors_callback);

  attitude_topic = drone_name + String("/attitude");
  attitude_sub = new ros::Subscriber<mavros_msgs::AttitudeTarget>(attitude_topic.c_str(), attitude_callback);

  battery_topic = drone_name + String("/battery");                         /* Update topic name */
  battery_pub = new ros::Publisher(battery_topic.c_str(), &battery_msg);    /* Instantiate publisher */
  nh.advertise(*battery_pub); 

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
  nh.subscribe(*led_sub);
  nh.subscribe(*motors_sub);
  nh.subscribe(*attitude_sub);
  
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
  /*
  Trim_Roll = -125; // -1750, 1750
  Trim_Pitch = 250; // -1750, 1750
  Trim_Yaw = 250;  // -1750, 1750
  */

  delay(1000);
}
/*********************************************************/



/*********************************************************
 * ARDUINO LOOP FUNCTION
*********************************************************/
void loop() {
  timer_freq = millis();
  /* ROS INFOS */
  if ((millis() - log_timer_ros) > 5000 && !gyro_calibration_status && !mag_calibration_status) {
    log_timer_ros = millis();
    sprintf(mbuf, "\33[96m[%s] Conected at time: %d, loop_freq: %f, battery: %f\33[0m", drone_name.c_str(), millis(), loop_freq, (float)analogRead(A0) * 5.5);
    nh.loginfo(mbuf);
    sprintf(mbuf, "\33[96m[%s] ArmControl: %d, landingoff: %d, stopFlightControl: %d\33[0m", drone_name.c_str(), armControl, landingOff, stopFlightControl);
    nh.loginfo(mbuf);
    sprintf(mbuf, "\33[96m[%s] throttle: %f, motorFL: %f, motorFR: %f, motorRL: %f, motorRR: %f\33[0m", drone_name.c_str(), throttle, motorFL, motorFR, motorRL, motorRR);
    nh.loginfo(mbuf);
    
  }

  /* ROS Loop */
  if (millis() - timer_ros > rate_ros) {
    update_battery(); 
    gyro_calibration_update();
    mag_calibration_update();
    timer_ros = millis();
    nh.spinOnce();
  }
  if (enable_motors_only){
      pwm_set_duty((pwmMotorFL_), 0);
      pwm_set_duty((pwmMotorFR_), 3);
      pwm_set_duty((pwmMotorRR_), 2);
      pwm_set_duty((pwmMotorRL_), 1);
      pwm_start();

  }else{
    //mainLoop();
    //modeControl();
    if (!gyro_calibration_status && !mag_calibration_status){
      FlightControl(); 
    }
      

  }
    
    
    loop_freq = 1000.0/(millis() - timer_freq);
    timer_freq = millis();
}
/*********************************************************/


/*********************************************************
 * LED CALLBACK FUNCTION
*********************************************************/
void led_callback(const std_msgs::ColorRGBA& msg) {
  analogWrite(redLed, (int)msg.r);
  analogWrite(greenLed, (int)msg.g);
  analogWrite(blueLed, (int)msg.b);
  sprintf(mbuf, "\33[96m[Drone-1] Turning the leds on RGB(%d, %d, %d)...\33[0m", (int)msg.r, (int)msg.g, (int)msg.b);
  nh.loginfo(mbuf);
}
/*********************************************************/



/*********************************************************
 * ARM MOTORS CALLBACK FUNCTION
*********************************************************/
void arm_motors_callback(const std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {
  armControl = !armControl;
  analogWriteFreq(20000);
  if (armControl == 1){
    sprintf(mbuf, "\33[91m[Drone-1] Be careful! Motors are enabled!\33[0m");
//    for (int i = 0; i < 12; i++) {
//      analogWrite(14, i * 10);
//      analogWrite(15, i * 10);
//      analogWrite(12, i * 10);
//      analogWrite(13, i * 10);
//      delay(25);
//    }
//    analogWrite(14, 0);
//    analogWrite(15, 0);
//    analogWrite(12, 0);
//    analogWrite(13, 0);
  } else {
    enable_motors_only = false;
    throttle = 0;
    sprintf(mbuf, "\33[92m[Drone-1] Motors are disabled!\33[0m");
  }  
  analogWriteFreq(20000);
  nh.loginfo(mbuf);
}
/*********************************************************/



/*********************************************************
 * RC MOTORS CALLBACK
*********************************************************/
void motors_callback(const mavros_msgs::RCOut& msg){
  if (!armControl){
    sprintf(mbuf, "\33[93m[Drone-1] Motors are disarmed!\33[0m");
    nh.loginfo(mbuf);
    enable_motors_only = false;
    return;
  } 
  sprintf(mbuf, "\33[93m[Drone-1] Settings motors to: %d, %d, %d, %d \33[0m",msg.channels[0], msg.channels[1], msg.channels[2], msg.channels[3]);
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



/*********************************************************
 * ATTITUDE CONTROL CALLBACK
*********************************************************/
void attitude_callback(const mavros_msgs::AttitudeTarget& msg){
  enable_motors_only = false;
  if (!armControl){
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
  SetPoint[0] = map(msg.body_rate.x, -100, 100, -ROLL_LIMIT, ROLL_LIMIT);
  SetPoint[1] = map(msg.body_rate.y, -100, 100, -PITCH_LIMIT, PITCH_LIMIT);
  SetPoint[2] = map(msg.body_rate.z, -100, 100, -YAW_LIMIT, YAW_LIMIT);
}
/*********************************************************/



/*********************************************************
 * BATTERY STATUS PUBLISHER
*********************************************************/
void update_battery(void){
  battery_msg.voltage = analogRead(A0) * 5.5;
  battery_msg.remaining = battery_msg.voltage - 3000;
  battery_msg.header.stamp = nh.now();
  battery_pub->publish( &battery_msg );
}
/*********************************************************/




/*********************************************************
 * GYRO CALIBRATION SERVICE
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
    if (print_once_gyro){
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
  } else if (tnow > 10000 && tnow < 10500){
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
 * MAGNETOMETER CALIBRATION SERVICE
*********************************************************/
void mag_calibration_callback(const std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {  
  sprintf(mbuf, "\33[93m[%s] Starting magnetometer calibration...\33[0m", drone_name.c_str());
  nh.loginfo(mbuf);
  mag_calibration_timer = millis();
  mag_calibration_status = true;
}

void mag_calibration_update(){
  if (!mag_calibration_status) return;

  double tnow = millis() - mag_calibration_timer;
  
  static float attitudeFake[3] = {0};
  static float rateFake[3] = {0};
  static float magOffsetMotorFnlFake[3] = {0};

  static bool print_once_mag = true;
  static bool print_once_mag_l1 = true;
  static bool print_once_mag_l2 = true;
  
  if (tnow < 500){
    print_once_mag = true;
    print_once_mag_l1 = true;
    print_once_mag_l2 = true;
    ahrs.magOffsetMotorCount = 0;
    ahrs.magOffsetMotorCount2 = 0;
  } else if (tnow < 3000) {  
    if (print_once_mag){
      print_once_mag = false;
      ahrs.magOffset[0] = 0;
      ahrs.magOffset[1] = 0;
      ahrs.magOffset[2] = 0;
      ahrs.magOffsetMotorFnl[0] = 0;
      ahrs.magOffsetMotorFnl[1] = 0;
      ahrs.headingMag(rateFake, attitudeFake, ahrs.degree , 0.0);
      ahrs.minX = ahrs.degree[1];
      ahrs.maxX = ahrs.degree[1];
      ahrs.minY = ahrs.degree[2];
      ahrs.maxY = ahrs.degree[2];
      sprintf(mbuf, "\33[91m[%s] Turn the drone three times C.W.!\33[0m", drone_name.c_str());
      nh.loginfo(mbuf);
    }
  } else if (tnow > 3000 && tnow < 15000) {
      ahrs.I2Cread(MPU9250_ADDRESS, 0x3B, 14, ahrs.Buf);
      ahrs.headingMag(rateFake, attitudeFake, ahrs.degree , 0);
      if (ahrs.degree[1] < ahrs.minX) ahrs.minX = ahrs.degree[1];
      if (ahrs.degree[1] > ahrs.maxX) ahrs.maxX = ahrs.degree[1];
      if (ahrs.degree[2] < ahrs.minY) ahrs.minY = ahrs.degree[2];
      if (ahrs.degree[2] > ahrs.maxY) ahrs.maxY = ahrs.degree[2];  
      sprintf(mbuf, "\33[93m[%s] MagX: (%f, %f) MagY: (%f, %f) DegreeXY: (%f %f) \33[0m", drone_name.c_str(), ahrs.minX, ahrs.maxX, ahrs.minY, ahrs.maxY, ahrs.degree[1], ahrs.degree[2]);
      nh.loginfo(mbuf);
      print_once_mag = true;
  } else if (tnow > 15000 && tnow < 30000) {
      if (print_once_mag){
        ahrs.magOffset[0] = (ahrs.maxX + ahrs.minX) / 2;
        ahrs.magOffset[1] = (ahrs.maxY + ahrs.minY) / 2;
        
        ahrs.mag_scale[0]  = (ahrs.maxX - ahrs.minX) / 2; // get average x axis max chord length in counts
        ahrs.mag_scale[1]  = (ahrs.maxY - ahrs.minY) / 2; // get average y axis max chord length in counts
        
        float avg_rad  = (ahrs.mag_scale[0] + ahrs.mag_scale[1]) / 2;
        
        ahrs.destination2[0] = avg_rad / ((float)ahrs.mag_scale[0]) ;
        ahrs.destination2[1] = avg_rad / ((float)ahrs.mag_scale[1]) ;

        sprintf(mbuf, "\33[92m[%s] Mag Offset: %d, %d, %d\33[0m", drone_name.c_str(), ahrs.magOffset[0], ahrs.magOffset[1], ahrs.magOffset[2]);
        nh.loginfo(mbuf);

        sprintf(mbuf, "\33[91m[%s] Stop Moving! Waiting for the rest of the process.\33[0m", drone_name.c_str());
        nh.loginfo(mbuf);
        print_once_mag = false;
      }
      ahrs.headingMag(rateFake, attitudeFake, ahrs.degree , 0 );
      if (ahrs.degree_[2] != ahrs.degree[2] && ahrs.degree_[1] != ahrs.degree[1] ) {
        ahrs.magOffsetMotorCount += 1.0;
        ahrs.magOffsetMotor[0] += ahrs.degree[1];
        ahrs.magOffsetMotor[1] += ahrs.degree[2];
        ahrs.degree_[1] = ahrs.degree[1];
        ahrs.degree_[2] = ahrs.degree[2];
      }
  } else if (tnow > 30000 && tnow < 33000) {
      if (print_once_mag_l1){
        ahrs.sumMagOffsetMotor[0] = ahrs.magOffsetMotor[0] / ahrs.magOffsetMotorCount;
        ahrs.sumMagOffsetMotor[1] = ahrs.magOffsetMotor[1] / ahrs.magOffsetMotorCount;
        sprintf(mbuf, "\33[93m[%s] SumMag: (%f, %f) with %d samples.\33[0m", drone_name.c_str(), ahrs.sumMagOffsetMotor[0], ahrs.sumMagOffsetMotor[1], (int)ahrs.magOffsetMotorCount);
        nh.loginfo(mbuf);
        if (ahrs.magOffsetMotorCount < 190.0){
          sprintf(mbuf, "\33[91m[%s] Too few samples (Only %d)! Calibrationg failed!\33[0m", drone_name.c_str(), (int)ahrs.magOffsetMotorCount);
          nh.loginfo(mbuf);
          mag_calibration_status = false;
          return;
        } else {
          sprintf(mbuf, "\33[92m[%s] Be careful! Checking the mag with the motors on.\33[0m", drone_name.c_str());
          nh.loginfo(mbuf);
        }
        print_once_mag_l1 = false;
      }
  } else if (tnow > 33000 && tnow < 34000) {
      if (print_once_mag_l2){
        analogWriteFreq(20000);
        for (int i = 0; i < 30; i++) {
          analogWrite(14, i * 10);
          analogWrite(15, i * 10);
          analogWrite(12, i * 10);
          analogWrite(13, i * 10);
          delay(25);
        }
        print_once_mag_l2 = false;
      }
      analogWrite(14, 700);
      analogWrite(15, 700);
      analogWrite(12, 700);
      analogWrite(13, 700);
      ahrs.headingMag(rateFake, attitudeFake, ahrs.degree , 0.0);
      if (ahrs.degree_[0] != ahrs.degree[0] && ahrs.degree_[1] != ahrs.degree[1] ) {
        ahrs.magOffsetMotorCount2 += 1.0;
        ahrs.degree_[1] = ahrs.degree[1]; // try with the sum
        ahrs.degree_[2] = ahrs.degree[2];
      }
  } else if (tnow > 34000) {
      analogWrite(14, 0);
      analogWrite(15, 0);
      analogWrite(12, 0);
      analogWrite(13, 0);

      ahrs.sumMagOffsetMotor2[0] = ahrs.degree[1];
      ahrs.sumMagOffsetMotor2[1] = ahrs.degree[2];
      sprintf(mbuf, "\33[93m[%s] SumMag2: (%f, %f) with %d samples.\33[0m", drone_name.c_str(), ahrs.sumMagOffsetMotor2[0], ahrs.sumMagOffsetMotor2[1], (int)ahrs.magOffsetMotorCount2);
      nh.loginfo(mbuf);
      if (ahrs.magOffsetMotorCount2 < 5){
        sprintf(mbuf, "\33[91m[%s] Too few samples (Only %d)! Calibrationg failed!\33[0m", drone_name.c_str(), (int)ahrs.magOffsetMotorCount2);
        nh.loginfo(mbuf);
        mag_calibration_status = false;
        return;
      }
      
      ahrs.magOffsetMotorEpr[0] = (int16_t)(ahrs.sumMagOffsetMotor[0] - ahrs.sumMagOffsetMotor2[0]);
      ahrs.magOffsetMotorEpr[1] = (int16_t)(ahrs.sumMagOffsetMotor[1] - ahrs.sumMagOffsetMotor2[1]);

      sprintf(mbuf, "\33[93m[%s] MagEpr: (%d, %d)\33[0m", drone_name.c_str(), ahrs.magOffsetMotorEpr[0], ahrs.magOffsetMotorEpr[1]);
      nh.loginfo(mbuf);

      EEPROM.begin(512);

      EEPROM.write(12, highByte(int(ahrs.destination2[0] * 100)));
      EEPROM.write(13, lowByte(int(ahrs.destination2[0] * 100)));
    
      EEPROM.write(14, highByte(int(ahrs.destination2[1] * 100)));
      EEPROM.write(15, lowByte(int(ahrs.destination2[1] * 100)));
    
      EEPROM.write(10, highByte(ahrs.magOffset[0]));
      EEPROM.write(11, lowByte(ahrs.magOffset[0]));
    
      EEPROM.write(20, highByte(ahrs.magOffset[1]));
      EEPROM.write(21, lowByte(ahrs.magOffset[1]));
    
      EEPROM.write(30, highByte(ahrs.magOffset[2]));
      EEPROM.write(31, lowByte(ahrs.magOffset[2]));
    
      EEPROM.write(40, highByte(ahrs.magOffsetMotorEpr[0]));
      EEPROM.write(41, lowByte(ahrs.magOffsetMotorEpr[0]));
    
      EEPROM.write(50, highByte(ahrs.magOffsetMotorEpr[1]));
      EEPROM.write(51, lowByte(ahrs.magOffsetMotorEpr[1]));

      EEPROM.end();

      sprintf(mbuf, "\33[92m[%s] Magnetometer is calibrated!\33[0m", drone_name.c_str());
      nh.loginfo(mbuf);
      sprintf(mbuf, "\33[92m[%s] Destination: (%f, %f)\33[0m", drone_name.c_str(), ahrs.destination2[0], ahrs.destination2[1]);
      nh.loginfo(mbuf);
      sprintf(mbuf, "\33[92m[%s] magOffset: (%d, %d, %d)\33[0m", drone_name.c_str(), ahrs.magOffset[0], ahrs.magOffset[1], ahrs.magOffset[2]);
      nh.loginfo(mbuf);
      sprintf(mbuf, "\33[92m[%s] magOffsetMotorEpr: (%d, %d)\33[0m", drone_name.c_str(), ahrs.magOffsetMotorEpr[0], ahrs.magOffsetMotorEpr[1]);
      nh.loginfo(mbuf);
      mag_calibration_status = false;
  }

}
/*********************************************************/


/*********************************************************
 * E N D
*********************************************************/

