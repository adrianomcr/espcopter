#include "AHRS.h"
#include "Wire.h"

int previousDegree;
float angle = 0;
float heading = 0;
void AHRS::Initialize()
{
  // Start I2C Communication
  //Wire.begin();
  Wire.pins(4, 5);
  Wire.begin(4, 5);
  Wire.setClock(400000L);
  //Wire.setClockStretchLimit(500L);

  // Set accelerometers low pass filter at 5Hz
  writeBIT(29, 0x06, MPU9250_ADDRESS);
  // Set gyroscope low pass filter at 5Hz
  writeBIT(26, 0x06, MPU9250_ADDRESS);
  // Configure gyroscope range
  writeBIT(27, GYRO_FULL_SCALE_2000_DPS, MPU9250_ADDRESS);
  // Configure accelerometers range
  writeBIT(28, ACC_FULL_SCALE_2_G, MPU9250_ADDRESS);
  // Set by pass mode for the magnetometers
  writeBIT(0x37, 0x02, MPU9250_ADDRESS);
  // Request continuous magnetometer measurements in 16 bits
  //writeBIT(0x0A, 0x16,MAG_ADDRESS);
  delay(20);

  writeBIT(0x0A, 0x00 , MAG_ADDRESS); // Power down magnetometer
  delay(20);
  writeBIT(0x0A  , 0x0F, MAG_ADDRESS); // Enter Fuse ROM access mode
  delay(20);
  uint8_t rawData[3];
  I2Cread(MAG_ADDRESS, 0x10, 3, rawData);
  destination[0] =  (float)(rawData[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
  destination[1] =  (float)(rawData[1] - 128) / 256. + 1.;
  destination[2] =  (float)(rawData[2] - 128) / 256. + 1.; //28 , 238 , 254 , 0.61 , 1.43 , 1.49

  delay(20);
  writeBIT(0x0A, 0x00, MAG_ADDRESS); // Power down magnetometer
  delay(20);
  
  // Configure the magnetometer for continuous read and highest resolution
  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
  writeBIT(0x0A, Mscale << 4 | Mmode, MAG_ADDRESS); // Set magnetometer data resolution and sample ODR
  delay(20);

  EEPROM.begin(512);
  
  if (EEPROM.read(4) == 1) {
    MEAN_GYRO[0] = (int32_t)-EEPROM.read(1);
  } else {
    MEAN_GYRO[0] = (int32_t)EEPROM.read(1);
  }
  if (EEPROM.read(5) == 1) {
    MEAN_GYRO[1] = (int32_t)-EEPROM.read(2);
  } else {
    MEAN_GYRO[1] = (int32_t)EEPROM.read(2);
  }
  if (EEPROM.read(6) == 1) {
    MEAN_GYRO[2] = (int32_t)-EEPROM.read(3);
  } else {
    MEAN_GYRO[2] = (int32_t)EEPROM.read(3);
  }

  magOffset[0] = (int16_t) word(EEPROM.read(10), EEPROM.read(11));
  magOffset[1] = (int16_t) word(EEPROM.read(20), EEPROM.read(21));
  magOffset[2] = (int16_t) word(EEPROM.read(30), EEPROM.read(31));

  magOffsetMotorEpr[0] = (int16_t) word(EEPROM.read(40), EEPROM.read(41));
  magOffsetMotorEpr[1] = (int16_t) word(EEPROM.read(50), EEPROM.read(51));

  magOffsetMotorEpr[2] = (int16_t) word(EEPROM.read(45), EEPROM.read(46));
  magOffsetMotorEpr[3] = (int16_t) word(EEPROM.read(55), EEPROM.read(56));

  destination2[0] = (float) word(EEPROM.read(12), EEPROM.read(13)) / 100;
  destination2[1] = (float) word(EEPROM.read(14), EEPROM.read(15)) / 100;

  EEPROM.end();
}


void AHRS::writeBIT(int addr, int data, int adrrTrans) {
  Wire.beginTransmission(adrrTrans);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();
}

void AHRS::I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data) {
  // Set register address
  Wire.beginTransmission(Address);
  Wire.write(Register);
  Wire.endTransmission();
  // Read Nbytes
  Wire.requestFrom(Address, Nbytes);
  uint8_t index = 0;
  while (Wire.available())
    Data[index++] = Wire.read();
}

float AHRS::rad2deg(float rad) {
  float output = rad * (180.0 / PI);
  return output;
}

float AHRS::deg2rad(float deg) {
  float output = deg * (PI / 180.0);
  return output;
}




//[Adriano]
void AHRS::EKF_getStates(float states[9]) {

  for(i=0; i<9; i++){
    states[i] = EKF_states[i];
  }
}



//[Adriano]
void AHRS::IMU_update() {

    // Get Raw Data From IMU
    I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);
    
      // Accelerometer
    accel[0] = (Buf[0] << 8 | Buf[1]);
    accel[1] = (Buf[2] << 8 | Buf[3]);
    accel[2] = Buf[4] << 8 | Buf[5];
    //Convert to m/s2 and correct the frame (x forward and z up)
    accel_si[0] = (accel[1] / 1685.0); //fine tunning
    accel_si[1] = -(accel[0] / 1640.3); //fine tunning
    accel_si[2] = (accel[2] / 1461.4); //fine tunning
    
    // Gyroscope
    gyro_int[0] = (Buf[8] << 8 | Buf[9]) - MEAN_GYRO[0];
    gyro_int[1] = (Buf[10] << 8 | Buf[11]) - MEAN_GYRO[1];
    gyro_int[2] = Buf[12] << 8 | Buf[13] - MEAN_GYRO[2];
    //Convert to rad/s and correct the frame (x forward and z up)
    float alpha_gyro = 0.99;
    gyro[0] = (1-alpha_gyro)*gyro[0] + alpha_gyro*(gyro_int[1] / GYRO_LSB) * DEG2RAD;
    gyro[1] = (1-alpha_gyro)*gyro[1] + -alpha_gyro*(gyro_int[0] / GYRO_LSB) * DEG2RAD;
    gyro[2] = (1-alpha_gyro)*gyro[2] + alpha_gyro*(gyro_int[2] / GYRO_LSB) * DEG2RAD;


}




//[Adriano]
void AHRS::EKF_prediction() {

  timenow = millis();
  dt = (float)(timenow - timeprev) / 978.;
  timeprev = timenow;

  if (dt > -1) {
    
//    // Get Raw Data From IMU
//    I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);
//    
//      // Accelerometer
//    accel[0] = (Buf[0] << 8 | Buf[1]);
//    accel[1] = (Buf[2] << 8 | Buf[3]);
//    accel[2] = Buf[4] << 8 | Buf[5];
//    //Convert to m/s2 and correct the frame (x forward and z up)
//    accel_si[0] = (accel[1] / 1685.0); //fine tunning
//    accel_si[1] = -(accel[0] / 1640.3); //fine tunning
//    accel_si[2] = (accel[2] / 1461.4); //fine tunning
//    
//    // Gyroscope
//    gyro_int[0] = (Buf[8] << 8 | Buf[9]) - MEAN_GYRO[0];
//    gyro_int[1] = (Buf[10] << 8 | Buf[11]) - MEAN_GYRO[1];
//    gyro_int[2] = Buf[12] << 8 | Buf[13] - MEAN_GYRO[2];
//    //Convert to rad/s and correct the facrorate_controlrame (x forward and z up)
//    float alpha_gyro = 0.3;
//    gyro[0] = (1-alpha_gyro)*gyro[0] + alpha_gyro*(gyro_int[1] / GYRO_LSB) * DEG2RAD;
//    gyro[1] = (1-alpha_gyro)*gyro[1] + -alpha_gyro*(gyro_int[0] / GYRO_LSB) * DEG2RAD;
//    gyro[2] = (1-alpha_gyro)*gyro[2] + alpha_gyro*(gyro_int[2] / GYRO_LSB) * DEG2RAD;

    //filter gyro and acc?


//    float phi = EKF_states[6];
//    float theta = EKF_states[7];
//    float psi = EKF_states[8];
//
//    float phi_d = gyro[0] + sin(phi)*tan(theta)*gyro[1] + cos(phi)*tan(theta)*gyro[2];
//    float theta_d = cos(phi)*gyro[1] -sin(phi)*gyro[2];
//    float psi_d = sin(phi)/cos(theta)*gyro[1] + cos(phi)/cos(theta)*gyro[2];

    float qw = EKF_states[6];
    float qx = EKF_states[7];
    float qy = EKF_states[8];
    float qz = EKF_states[9];

    float qw_d = 0.5*(-qx*gyro[0] - qy*gyro[1] - qz*gyro[2]);
    float qx_d = 0.5*(qw*gyro[0] - qz*gyro[1] + qy*gyro[2]);
    float qy_d = 0.5*(qz*gyro[0] + qw*gyro[1] - qx*gyro[2]);
    float qz_d = 0.5*(-qy*gyro[0] + qx*gyro[1] + qw*gyro[2]);

//d_quat = 0.5*[-qx*wx - qy*wy - qz*wz;
//              qw*wx - qz*wy + qy*wz;
//              qz*wx + qw*wy - qx*wz;
//              -qy*wx + qx*wy + qw*wz];


    float quat[4] = {EKF_states[6], EKF_states[7], EKF_states[8], EKF_states[9]};
    float ang[3] = {0,0,0};
    quat2eul(quat, ang);
    //Compute velocity in world frame
    float vel_w[3];
    vel_w[0] = (cos(ang[1])*cos(ang[2]))*EKF_states[3] + (sin(ang[0])*sin(ang[1])*cos(ang[2])-cos(ang[0])*sin(ang[2]))*EKF_states[4] + (cos(ang[0])*sin(ang[1])*cos(ang[2])+sin(ang[0])*sin(ang[2]))*EKF_states[5];
    vel_w[1] = (cos(ang[1])*sin(ang[2]))*EKF_states[3] + (sin(ang[0])*sin(ang[1])*sin(ang[2])+cos(ang[0])*cos(ang[2]))*EKF_states[4] + (cos(ang[0])*sin(ang[1])*sin(ang[2])-sin(ang[0])*cos(ang[2]))*EKF_states[5];
    vel_w[2] = (-sin(ang[1]))*EKF_states[3] + (sin(ang[0])*cos(ang[1]))*EKF_states[4] + (cos(ang[0])*cos(ang[1]))*EKF_states[5];
    //CONSIDER USING THE quat_vec_transform INSTEAD


    
    //Propagate positions
    EKF_states[0] = EKF_states[0] + vel_w[0]*dt;
    EKF_states[1] = EKF_states[1] + vel_w[1]*dt;
    EKF_states[2] = EKF_states[2] + vel_w[2]*dt;
    //Propagate velocities
    EKF_states[3] = EKF_states[3] + 0*dt;
    EKF_states[4] = EKF_states[4] + 0*dt;
    EKF_states[5] = EKF_states[5] + 0*dt;
//    //Propagate angles
//    EKF_states[6] = EKF_states[6] + phi_d*dt;
//    EKF_states[7] = EKF_states[7] + theta_d*dt;
//    EKF_states[8] = EKF_states[8] + psi_d*dt;
    //Propagate quaternion
    EKF_states[6] = EKF_states[6] + qw_d*dt;
    EKF_states[7] = EKF_states[7] + qx_d*dt;
    EKF_states[8] = EKF_states[8] + qy_d*dt;
    EKF_states[9] = EKF_states[9] + qz_d*dt;


//    JT << 1.0, sin(phi)*tan(theta), cos(phi)*tan(theta),
//          0.0, cos(phi), -sin(phi),
//          0.0, sin(phi)/cos(theta), cos(phi)/cos(theta);
    
// R_bw << (cos(theta)*cos(psi)), (sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)), (cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)),
//          (cos(theta)*sin(psi)), (sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)), (cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)),
//          (-sin(theta)), (sin(phi)*cos(theta)), (cos(phi)*cos(theta));

    //------------------------------------------------------------------
    //Perform velocity correction with an inderect measurement from IMU?
    //Estimate air resistance model
    //------------------------------------------------------------------

  }
 
}


//[Adriano]
unsigned int last_mocap = micros();
float last_pos[3] = {0, 0, 0};
float dt_vel = 0;
void AHRS::EKF_update(float z_pos[3], float z_quat[4] ) {
  dt_vel = (float) (micros()-last_mocap)/1000000.0;
  last_mocap = micros();
//  if(dt<0 || dt>0.1){
//    return;
//  }
//  sprintf(mbuf, "\33[46m[%s] Doing update\33[0m", drone_name.c_str());
//  nh.loginfo(mbuf);


  float quat[4] = {EKF_states[6], EKF_states[7], EKF_states[8], EKF_states[9]};
  float ang[3] = {0,0,0};
  quat2eul(quat, ang);

  float z_ang[3] = {0,0,0};
  quat2eul(z_quat, z_ang);
//  float z_phi = z_ang[0];
//  float z_theta = z_ang[1];
//  float z_psi = z_ang[2];
  
//  float sinr_cosp = +2.0 * (z_quat[0] * z_quat[1] + z_quat[2] * z_quat[3]);
//  float cosr_cosp = +1.0 - 2.0 * (z_quat[1] * z_quat[1] + z_quat[2] * z_quat[2]);
//  z_phi = atan2(sinr_cosp, cosr_cosp);
//
//  // pitch (y-axis rotation)
//  float sinp = +2.0 * (z_quat[0] * z_quat[2] - z_quat[3] * z_quat[1]);
//  if (fabs(sinp) >= 1)
//    z_theta = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
//  else
//    z_theta = asin(sinp);
//
//  // yaw (z-axis rotation)
//  float siny_cosp = +2.0 * (z_quat[0] * z_quat[3] + z_quat[1] * z_quat[2]);
//  float cosy_cosp = +1.0 - 2.0 * (z_quat[2] * z_quat[2] + z_quat[3] * z_quat[3]);
//  z_psi = atan2(siny_cosp, cosy_cosp);


  
  float z_vel_w[3];
  z_vel_w[0] = (z_pos[0]-last_pos[0])/dt_vel;
  z_vel_w[1] = (z_pos[1]-last_pos[1])/dt_vel;
  z_vel_w[2] = (z_pos[2]-last_pos[2])/dt_vel;
  last_pos[0] = z_pos[0];
  last_pos[1] = z_pos[1];
  last_pos[2] = z_pos[2];

float z_vel_b[3];
float phi = z_ang[0];
float theta = z_ang[1];
float psi = z_ang[2];
//z_vel_b[0] = (cos(theta)*cos(psi))*z_vel_w[0] + (sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))*z_vel_w[1] + (cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))*z_vel_w[2];
//z_vel_b[1] = (cos(theta)*sin(psi))*z_vel_w[0] + (sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi))*z_vel_w[1] + (cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))*z_vel_w[2];
//z_vel_b[2] = (-sin(theta))*z_vel_w[0] + (sin(phi)*cos(theta))*z_vel_w[1] + (cos(phi)*cos(theta))*z_vel_w[2];
z_vel_b[0] = (cos(theta)*cos(psi))*z_vel_w[0] + (cos(theta)*sin(psi))*z_vel_w[1] + (-sin(theta))*z_vel_w[2];
z_vel_b[1] = (sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))*z_vel_w[0] + (sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi))*z_vel_w[1] + (sin(phi)*cos(theta))*z_vel_w[2];
z_vel_b[2] = (cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))*z_vel_w[0] + (cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))*z_vel_w[1] + (cos(phi)*cos(theta))*z_vel_w[2];
//CONSIDER USING THE quat_vec_transform INSTEAD


  // R_bw << (cos(theta)*cos(psi)), (sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)), (cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)),
//          (cos(theta)*sin(psi)), (sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)), (cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)),
//          (-sin(theta)), (sin(phi)*cos(theta)), (cos(phi)*cos(theta));




  float K_pos = 0.4;
  EKF_states[0] = EKF_states[0] + K_pos*(z_pos[0]-EKF_states[0]);
  EKF_states[1] = EKF_states[1] + K_pos*(z_pos[1]-EKF_states[1]);
  EKF_states[2] = EKF_states[2] + K_pos*(z_pos[2]-EKF_states[2]);


  float K_vel = 0.2;
  EKF_states[3] = EKF_states[3] + K_vel*(z_vel_b[0]-EKF_states[3]);
  EKF_states[4] = EKF_states[4] + K_vel*(z_vel_b[1]-EKF_states[4]);
  EKF_states[5] = EKF_states[5] + K_vel*(z_vel_b[2]-EKF_states[5]);

  
//  float K_angles = 0.4;
//  EKF_states[6] = EKF_states[6] + K_angles*sin(z_phi-EKF_states[6]);
//  EKF_states[7] = EKF_states[7] + K_angles*sin(z_theta-EKF_states[7]);
//  EKF_states[8] = EKF_states[8] + K_angles*sin(z_psi-EKF_states[8]);
  float K_angles = 0.4;
  ang[0] = ang[0] + K_angles*sin(z_ang[0]-ang[0]);
  ang[1] = ang[1] + K_angles*sin(z_ang[1]-ang[1]);
  ang[2] = ang[2] + K_angles*sin(z_ang[2]-ang[2]);
  eul2quat(ang, quat);
  EKF_states[6] = quat[0];
  EKF_states[7] = quat[1];
  EKF_states[8] = quat[2];
  EKF_states[9] = quat[3];
  //CONSIDER DOING THIS UPDATE DIRECTLY WITH QUATERNIONS






  //This function should be called every time a pose data is received

  //get the measured data

  //compute Kalman gain

  //Correct states

  //Correct covariance

  
}



//#define Kp_wx 18//30
//#define Kp_wy 18//30
//#define Kp_wz 18//30
//#define Ki_wx 0.05*0//8
//#define Ki_wy 0.05*0//8
//#define Ki_wz 0.05*0//8
#define Jx 0.00003
#define Jy 0.00003
#define Jz 0.000045

//#define Kp 1.2e-4
//#define Kp 1.9e-4 //increase this value to avoid drone flying too much
#define d 0.0315 // 63mm/2
#define Kd 1.0e-6 //torque z

////The constants below correspond to a linearization of the map pwm -> thrust (f_i = c1 + c2*pwm)
#define c1 0.0352
#define c2 0.94079e-04

//#define bias_pwm_0 -15
//#define bias_pwm_1 -15 -10
//#define bias_pwm_2 -10
//#define bias_pwm_3 0
#define bias_pwm_0 0
#define bias_pwm_1 0
#define bias_pwm_2 0
#define bias_pwm_3 0

#define K_tau 10.0
#define m 0.0395

#define umin 300
#define umax 850



#define N_BUF_DERIV 50
//#define MIN_TIME_DERIV 0.002*1000*1000  // in microseconds
#define MIN_TIME_DERIV 2000*5  // in microseconds
unsigned long buf_t[N_BUF_DERIV];
float buf_ewx[N_BUF_DERIV];
float buf_ewy[N_BUF_DERIV];
float buf_ewz[N_BUF_DERIV];
bool start_deriv = false;
int i_ref_deriv = 0;


void AHRS::acrorate_control(float tau_ref, float omega_ref[3], float dt_in, float u_out[4]){

  
  float err_omega[3] = {omega_ref[0]-gyro[0], omega_ref[1]-gyro[1], omega_ref[2]-gyro[2]};


////  float diff_err_omega[3] = {(err_omega[0]-last_err_omega[0])/dt_in, (err_omega[1]-last_err_omega[1])/dt_in, (err_omega[2]-last_err_omega[2])/dt_in};
//  float diff_err_omega[3];
//  float alpha_dif = 0.3-0.1;
//  diff_err_omega[0] = (1-alpha_dif)*diff_err_omega[0] + alpha_dif*(err_omega[0]-last_err_omega[0])/(dt_in/2.0);
//  diff_err_omega[1] = (1-alpha_dif)*diff_err_omega[1] + alpha_dif*(err_omega[1]-last_err_omega[1])/(dt_in/2.0);
//  diff_err_omega[2] = (1-alpha_dif)*diff_err_omega[2] + alpha_dif*(err_omega[2]-last_err_omega[2])/(dt_in/2.0); 
//  last_err_omega[0] = err_omega[0];
//  last_err_omega[1] = err_omega[1];
//  last_err_omega[2] = err_omega[2];

  //Estimate the derivative of the error signal
  float diff_err_omega[3];
  compute_error_derivative(err_omega, diff_err_omega);

  if (tau_ref > 0.2){
    int_err_omega[0] = int_err_omega[0] + err_omega[0]*dt_in;
    int_err_omega[1] = int_err_omega[1] + err_omega[1]*dt_in;
    int_err_omega[2] = int_err_omega[2] + err_omega[2]*dt_in;
  }


  float cross_omega[3];
  float J_omega[3] = {Jx*gyro[0], Jy*gyro[1], Jz*gyro[2]};
  cross(gyro, J_omega, cross_omega);


  float T_desired[3];
//  T_desired[0] = Jx*(cross_omega[0] + Kp_wx*err_omega[0] + Ki_wx*int_err_omega[0]);
//  T_desired[1] = Jy*(cross_omega[1] + Kp_wy*err_omega[1] + Ki_wy*int_err_omega[1]);
//  T_desired[2] = Jy*(cross_omega[2] + Kp_wz*err_omega[2] + Ki_wz*int_err_omega[2]);
  T_desired[0] = cross_omega[0] + Jx*(Kpwx*err_omega[0] + Kiwx*int_err_omega[0] + Kdwx*diff_err_omega[0] + omega_ref_dot[0]);
  T_desired[1] = cross_omega[1] + Jy*(Kpwy*err_omega[1] + Kiwy*int_err_omega[1] + Kdwy*diff_err_omega[1] + omega_ref_dot[1]);
  T_desired[2] = cross_omega[2] + Jz*(Kpwz*err_omega[2] + Kiwz*int_err_omega[2] + Kdwz*diff_err_omega[2] + omega_ref_dot[2]);
  T_desired[2] = -T_desired[2];

  float u0[4];

//  u0[0] = 1/(4*Kp)*tau_ref +(1/(4*Kp*d))*T_desired[0] -(1/(4*Kp*d))*T_desired[1] +(1/(4*Kd))*T_desired[2];
//  u0[1] = 1/(4*Kp)*tau_ref -(1/(4*Kp*d))*T_desired[0] -(1/(4*Kp*d))*T_desired[1] -(1/(4*Kd))*T_desired[2];
//  u0[2] = 1/(4*Kp)*tau_ref -(1/(4*Kp*d))*T_desired[0] +(1/(4*Kp*d))*T_desired[1] +(1/(4*Kd))*T_desired[2];
//  u0[3] = 1/(4*Kp)*tau_ref +(1/(4*Kp*d))*T_desired[0] +(1/(4*Kp*d))*T_desired[1] -(1/(4*Kd))*T_desired[2];

  // Check this new map: f_i = c1 + c2*u
  tau_ref = tau_ref + K_tau*(tau_ref - m*accel_si[2]);
  if(tau_ref > 0.46){
    tau_ref = 0.46;
  }
  u0[0] = 1/(4*c2)*(tau_ref-4*c1) +(1/(4*c2*d))*T_desired[0] -(1/(4*c2*d))*T_desired[1] +(1/(4*Kd))*T_desired[2];
  u0[1] = 1/(4*c2)*(tau_ref-4*c1) -(1/(4*c2*d))*T_desired[0] -(1/(4*c2*d))*T_desired[1] -(1/(4*Kd))*T_desired[2];
  u0[2] = 1/(4*c2)*(tau_ref-4*c1) -(1/(4*c2*d))*T_desired[0] +(1/(4*c2*d))*T_desired[1] +(1/(4*Kd))*T_desired[2];
  u0[3] = 1/(4*c2)*(tau_ref-4*c1) +(1/(4*c2*d))*T_desired[0] +(1/(4*c2*d))*T_desired[1] -(1/(4*Kd))*T_desired[2];

  u0[0] = u0[0] + bias_pwm_0;
  u0[1] = u0[1] + bias_pwm_1;
  u0[2] = u0[2] + bias_pwm_2;
  u0[3] = u0[3] + bias_pwm_3;


  //Map to the motors numbers considered by the espcopter and filter
  float acrorate_alpha = 0.9-0.2 + 0.29;
//  float acrorate_alpha = 0.4;
  u_motors[0] = (1-acrorate_alpha)*u_motors[0] + (acrorate_alpha)*u0[0];
  u_motors[1] = (1-acrorate_alpha)*u_motors[1] + (acrorate_alpha)*u0[2];
  u_motors[2] = (1-acrorate_alpha)*u_motors[2] + (acrorate_alpha)*u0[3];
  u_motors[3] = (1-acrorate_alpha)*u_motors[3] + (acrorate_alpha)*u0[1];

  //Saturate cmds
  for (int ii=0; ii<4; ii++){
    if(u_motors[ii]<umin){
      u_motors[ii] = umin;
    }
    if(u_motors[ii]>umax){
      u_motors[ii] = umax;
    }
  }

  
  u_out[0] = u_motors[0];
  u_out[1] = u_motors[1];
  u_out[2] = u_motors[2];
  u_out[3] = u_motors[3];

}







void AHRS::compute_error_derivative(float ew[3], float ew_deriv[3]){

  ew_deriv[0] = 0;
  ew_deriv[1] = 0;
  ew_deriv[2] = 0;

//Compute the derivative of the acro rate reference
  if(start_deriv){

    buf_t[i_ref_deriv] = micros();
    buf_ewx[i_ref_deriv] = ew[0];
    buf_ewy[i_ref_deriv] = ew[1];
    buf_ewz[i_ref_deriv] = ew[2];
    
    int j = i_ref_deriv;

    for (int i=1; i<N_BUF_DERIV; i++){
      j = i_ref_deriv-i;

      if(j<0){
        j = j + N_BUF_DERIV;
      }
      if(buf_t[i_ref_deriv]-buf_t[j] > MIN_TIME_DERIV){
        float dt_now = (float) (buf_t[i_ref_deriv]-buf_t[j])/(1000.0*1000.0);
        ew_deriv[0] = (buf_ewx[i_ref_deriv]-buf_ewx[j])/(dt_now);
        ew_deriv[1] = (buf_ewy[i_ref_deriv]-buf_ewy[j])/(dt_now);
        ew_deriv[2] = (buf_ewz[i_ref_deriv]-buf_ewz[j])/(dt_now);
        break;
      }
    }
    i_ref_deriv = i_ref_deriv + 1;
    if (i_ref_deriv == N_BUF_DERIV){
      i_ref_deriv = 0;
    }
  }
  else{

    buf_t[i_ref_deriv] = micros();
    buf_ewx[i_ref_deriv] = ew[0];
    buf_ewy[i_ref_deriv] = ew[1];
    buf_ewz[i_ref_deriv] = ew[2];
    i_ref_deriv = i_ref_deriv + 1;

    if (i_ref_deriv == N_BUF_DERIV){
      i_ref_deriv = 0;
      start_deriv = true;
    }
  }


  
}






//void AHRS::set_motors(float u[4]) {
//  int16_t pwmMotorFL_ = round(map(u[0], 0, 1023, 0, PWM_PERIOD)); //255
//  int16_t pwmMotorFR_ = round(map(u[3], 0, 1023, 0, PWM_PERIOD));
//  int16_t pwmMotorRL_ = round(map(u[2], 0, 1023, 0, PWM_PERIOD));
//  int16_t pwmMotorRR_ = round(map(u[1], 0, 1023, 0, PWM_PERIOD));
//
//  pwmMotorFL_ = constrain(pwmMotorFL_, 0, PWM_PERIOD);
//  pwmMotorFR_ = constrain(pwmMotorFR_, 0, PWM_PERIOD);
//  pwmMotorRL_ = constrain(pwmMotorRL_, 0, PWM_PERIOD);
//  pwmMotorRR_ = constrain(pwmMotorRR_, 0, PWM_PERIOD);
//
////  pwm_set_duty((pwmMotorFL_), 0);
////  pwm_set_duty((pwmMotorFR_), 3);
////  pwm_set_duty((pwmMotorRR_), 2);
////  pwm_set_duty((pwmMotorRL_), 1);
//
//}

//Function to convert a quaternion to euler angles
void AHRS::cross(float va[3], float vb[3], float vc[3]) {

  vc[0] = va[1]*vb[2] - va[2]*vb[1];
  vc[1] = va[2]*vb[0] - va[0]*vb[2];
  vc[2] = va[0]*vb[1] - va[1]*vb[0];
}


//Function to convert a quaternion to euler angles
void AHRS::quat2eul(float q_in[4], float ang_out[3]) {


  float sinr_cosp = +2.0 * (q_in[0] * q_in[1] + q_in[2] * q_in[3]);
  float cosr_cosp = +1.0 - 2.0 * (q_in[1] * q_in[1] + q_in[2] * q_in[2]);
  ang_out[0] = atan2(sinr_cosp, cosr_cosp);
  
  // pitch (y-axis rotation)
  float sinp = +2.0 * (q_in[0] * q_in[2] - q_in[3] * q_in[1]);
  if (fabs(sinp) >= 1)
    ang_out[1] = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    ang_out[1] = asin(sinp);
  
  // yaw (z-axis rotation)
  float siny_cosp = +2.0 * (q_in[0] * q_in[3] + q_in[1] * q_in[2]);
  float cosy_cosp = +1.0 - 2.0 * (q_in[2] * q_in[2] + q_in[3] * q_in[3]);
  ang_out[2] = atan2(siny_cosp, cosy_cosp);
  
}


//Function to convert euler angles to a quaternion
void AHRS::eul2quat(float ang_in[3], float q_out[4]) {

  float cy = cos(ang_in[2] * 0.5); //yaw
  float sy = sin(ang_in[2] * 0.5); //yaw
  float cp = cos(ang_in[1] * 0.5); //pitch
  float sp = sin(ang_in[1] * 0.5); //pitch
  float cr = cos(ang_in[0] * 0.5); //roll
  float sr = sin(ang_in[0] * 0.5); //roll
  
  q_out[0] = cy * cp * cr + sy * sp * sr;
  q_out[1] = cy * cp * sr - sy * sp * cr;
  q_out[2] = sy * cp * sr + cy * sp * cr;
  q_out[3] = sy * cp * cr - cy * sp * sr;

}


void AHRS::quat_normalize(float q_in[4], float q_out[4])
{
  float quat_norm = sqrt(q_in[0] * q_in[0] + q_in[1] * q_in[1] + q_in[2] * q_in[2] + q_in[3] * q_in[3]);
  q_out[0] = q_in[0] / norm;
  q_out[1] = q_in[1] / norm;
  q_out[2] = q_in[2] / norm;
  q_out[3] = q_in[3] / norm;
}

//Function to multiply two quaternions
void AHRS::quat_multiply(float qa[4], float qb[4], float q_out[4]) {

  q_out[0] = qa[0]*qb[0] - qa[1]*qb[1] -qa[2]*qb[2] -qa[3]*qb[3];
  q_out[1] = qa[0]*qb[1] + qa[1]*qb[0] +qa[2]*qb[3] -qa[3]*qb[2];
  q_out[2] = qa[0]*qb[2] - qa[1]*qb[3] +qa[2]*qb[0] +qa[3]*qb[1];
  q_out[3] = qa[0]*qb[3] + qa[1]*qb[2] -qa[2]*qb[1] +qa[3]*qb[0];

  quat_normalize(q_out,q_out);

}


//Function to perform a quaternion transformation to a vector
void AHRS::quat_vec_transform(float q_in[4], float v_in[3], float v_out[3]) {

  //quat_vec_transform(q_wb, v_b, v_w) == v_w <- R_bw*v_b

  float v_aux[4] = {0, v_in[0], v_in[1], v_in[2]};
  float q_star[4] = {q_in[0], -q_in[1], -q_in[2], -q_in[3]};

  quat_multiply(q_star,v_aux,v_aux);
  quat_multiply(v_aux,q_in,v_aux);
  
  v_out[0] = v_aux[1];
  v_out[1] = v_aux[2];
  v_out[2] = v_aux[3];

}




//------------------------------------------------

//------------------------------------------------


void AHRS::compute(float attitude[3], float rate[3], float attitudeRadian[3], float rateRadian[3] ) {
  timenow = millis();
  dt = (float)(timenow - timeprev) / 978.;
  timeprev = timenow;

  if (dt > -1) {
    // Get Raw Data From IMU
    I2Cread(MPU9250_ADDRESS, 0x3B, 14, Buf);
    

    // Accelerometer
    accel[0] = (Buf[0] << 8 | Buf[1]);
    accel[1] = (Buf[2] << 8 | Buf[3]);
    accel[2] = (Buf[4] << 8 | Buf[5]);

    
    // Gyroscope
    gyro_int[0] = (Buf[8] << 8 | Buf[9]) - MEAN_GYRO[0];
    gyro_int[1] = (Buf[10] << 8 | Buf[11]) - MEAN_GYRO[1];
    gyro_int[2] = Buf[12] << 8 | Buf[13] - MEAN_GYRO[2];

    for (i = 0; i < 3 ; i++) {
      // Get Gyro
      gyro[i] = (gyro_int[i] / GYRO_LSB) * DEG2RAD;
    }



    
    

    // Normalize Accelerometer
    normalize(accel_angle, accel);

    // Calculate deltaAngle
    gyro_angle_x = gyro_angle_x + (gyro_angle_y * gyro[2] - gyro_angle_z * gyro[1]) * dt;
    gyro_angle_y = gyro_angle_y + (gyro_angle_z * gyro[0] - gyro_angle_x * gyro[2]) * dt;
    gyro_angle_z = gyro_angle_z + (gyro_angle_z * gyro[1] - gyro_angle_y * gyro[0]) * dt;

    // Apply Complementary Filter
    // Apply when |a| - 1 < 0.10 to eliminate other forces
    if ((abs(norm) / 16384.) - 1 <= 0.15) { //0.15
      gyro_angle_x = A_A * gyro_angle_x + (1 - A_A) *  accel_angle[0];
      gyro_angle_y = A_A * gyro_angle_y + (1 - A_A) *  accel_angle[1];
      gyro_angle_z = A_A * gyro_angle_z + (1 - A_A) *  accel_angle[2];
    }

    // Convert
    attitude[0] = atan2(gyro_angle_y, sqrt(gyro_angle_x * gyro_angle_x + gyro_angle_z * gyro_angle_z)); // ROLL
    attitude[1] = atan2(gyro_angle_x, gyro_angle_z); // PITCH

    // Calculate Euler Rates
    rate[0] =  (gyro[0] + ( gyro[1] * attitude[0] * attitude[1]) + ( gyro[2] * (1 - (attitude[0] * attitude[0]) / 2) * attitude[1]));
    rate[1] =  (gyro[1] * (1 - ((attitude[0] * attitude[0]) / 2)) -  gyro[2] * attitude[0]);
    rate[2] = gyro[2];

    rateRadian[0] =  rate[0];
    rateRadian[1] =  rate[1];
    rateRadian[2] =  rate[2];

    rate[0] =  rate[0] * 100 * RAD2DEG;
    rate[1] =  rate[1] * 100 * RAD2DEG;
    rate[2] =  rate[2] * 100 * RAD2DEG;


    float alpha = 0.85;
    for (i = 0; i < 3 ; i++) {
      rate[i] = (alpha * (float)rate[i]) + ((1 - alpha) * (float)_rate[i]);
      _rate[i] = rate[i];
    }

    attitudeRadian[0] =  attitude[0];
    attitudeRadian[1] =  attitude[1];

    attitude[0] *= 100 * RAD2DEG;
    attitude[1] *= 100 * RAD2DEG;

    float alpha2 = 0.95;
    for (i = 0; i < 2 ; i++) {
      attitude[i] = (alpha2 * (float)attitude[i]) + ((1 - alpha2) * (float)_attitude[i]);
      _attitude[i] = attitude[i];
    }
  } else {
    Serial.print((float)dt);
    Serial.println("Error AHRS");
  }
}

void AHRS::normalize(float output[3], int16_t input[3])
{
  norm = sqrt(input[0] * input[0] + input[1] * input[1] + input[2] * input[2]);
  output[0] = input[0] / (float)norm;
  output[1] = input[1] / (float)norm;
  output[2] = input[2] / (float)norm;
}

void AHRS::setZero()
{
  gyro_angle_x = 0;
  gyro_angle_y = 0;
  gyro_angle_z = 0;
}

int16_t  AHRS::readTemp() {
  uint8_t rawData[2];  // x/y/z gyro register data stored here
  I2Cread(MPU9250_ADDRESS, 0x41, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  return ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a 16-bit value
}


void AHRS::headingMag(float attitude_rate[3], float  input[3], float degree[4], float throttle) {

  I2Cread(MAG_ADDRESS, 0x02, 1, &ST1);

  if (ST1 & 0x01) {  
    uint8_t Mag[7];
    I2Cread(MAG_ADDRESS, 0x03, 7, Mag);
    
    // Create 16 bits values from 8 bits data
    // Magnetometer - Raw Data
    mag_int[0] = (Mag[3] << 8 | Mag[2]);
    mag_int[1] = (Mag[1] << 8 | Mag[0]);
    mag_int[2] = (Mag[5] << 8 | Mag[4]);

    /*
      if(throttle == 0){
      magOffset3_ = magOffset[2];
      magCaCoefficient=0;
      }else{
      magCaCoefficient  =  map(100*constrain(magOffset3_  - magOffset[2],0,1500),0,150000,0.00,1000.00);
      magCaCoefficient = magCaCoefficient/100;
      }
    */

    if (input[0] < 0.1 && input[0] > -0.1 && input[1] > -0.1 && input[1] < 0.1 ) { //&&  throttle == 0
      if ((mag_int[2]* mRes * (int16_t) destination[2]) < 600) {
        magOffset[2] = 600 - (mag_int[2] * mRes * (int16_t) destination[2]);
      } else {
        magOffset[2] = -((mag_int[2] * mRes * (int16_t) destination[2]) - 600 );
      }
    }

    magOffsetMotorFnl2[0] =  magOffsetMotorEpr[0] * (int16_t)(((float)map(100 * constrain(throttle, 0, 700), 0, 70000, 0, 100)) / 100);
    magOffsetMotorFnl2[1] =  magOffsetMotorEpr[1] * (int16_t)(((float)map(100 * constrain(throttle, 0, 700), 0, 70000, 0, 100)) / 100);

    mag[0] = (mag_int[0] * mRes * (int16_t) destination[0]) - magOffset[0] + magOffsetMotorFnl2[0];
    mag[1] = (mag_int[1] * mRes * (int16_t) destination[1]) - magOffset[1] + magOffsetMotorFnl2[1];
    mag[2] = (mag_int[2] * mRes * (int16_t) destination[2]) + magOffset[2];

    // mag[0] *= destination2[0];
    // mag[1] *= destination2[1];

    degree[1] =  (float)mag[0];
    degree[2] =  (float)mag[1];
    degree[3] =  (float)mag[2];

    float roll;
    float pitch;

    pitch = asin(input[1]);
    roll = -asin(input[0]);

    float cosRoll = cos(roll);
    float sinRoll = sin(roll);
    float cosPitch = cos(pitch);
    float sinPitch = sin(pitch);

    float Xh =  mag[0] * cosPitch +  mag[2] * sinPitch;
    float Yh =  mag[0] * sinRoll * sinPitch +  mag[1] * cosRoll -  mag[2] * sinRoll * cosPitch;

    float heading1 = atan2(Yh, Xh);

    float declinationAngle2 = (4.0 + (26.0 / 60.0)) / (180 / M_PI);

    heading1 += declinationAngle2;
    // Correct for heading < 0deg and heading > 360deg
    if (heading1 < 0) {
      heading1 += 2 * PI;
    }
    if (heading1 > 2 * PI) {
      heading1 -= 2 * PI;
    }

    previousHeadingDegrees = headingDegrees;

    headingDegrees = heading1 * 180 / M_PI;


    unsigned long headingCurrentMillis = millis();
    /*
      if(throttle < 100){
      headingPreviousMillis = headingCurrentMillis;
      }

      if(headingCurrentMillis - headingPreviousMillis < 1000){
      headingDegreesDefoult = headingDegrees;
      }
    */

    if (throttle < 100) {
      headingDegreesDefoult = headingDegrees;
      HeadingLoop = 0;
    }

    int turnRight;
    int turnLeft ;

    if ( HeadingCount == true) {

      if (headingDegreesDefoult - previousHeadingDegrees < 30 &&  headingDegreesDefoult - previousHeadingDegrees > -30) {
        errorDirection = headingDegreesDefoult - previousHeadingDegrees;
      }


      if ( errorDirection > 0  ) {

        if (previousHeadingDegrees - headingDegrees  < -300 || turnLeft == 1) { // 360 to 0
          HeadingLoop = -360;
          turnLeft = 1;
        }

        if (previousHeadingDegrees - headingDegrees  > 300 && turnLeft == 1) { // 0 to 360
          turnLeft = 0;
          HeadingLoop = 0;
        }
      } else {

        if ( ( previousHeadingDegrees - headingDegrees  > 300 || turnRight == 1 ) ) { // 360 to 0
          HeadingLoop = 360;
          turnRight = 1;
        }

        if (previousHeadingDegrees - headingDegrees  < -300 && turnRight == 1) { // 0 to 360
          turnRight = 0;
          HeadingLoop = 0;
        }

      }
    }
    HeadingCount = true;


    degree[0] = (float)(headingDegrees  - headingDegreesDefoult + HeadingLoop); //MEAN_GYRO 
  }
}
