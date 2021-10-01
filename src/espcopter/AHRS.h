#include <Arduino.h>
#include "Parameter.h"
#include <Wire.h>
#include <EEPROM.h>

class AHRS {
  public:

    void Initialize();
    float rad2deg(float rad);
    float deg2rad(float deg);
    int16_t readTemp();
    void writeBIT(int addr, int data, int adrrTrans);
    void I2Cread(uint8_t Address, uint8_t Register, uint8_t Nbytes, uint8_t* Data);
    void compute(float attitude[3], float rate[3] , float attitudeRadian[3] , float rateRadian[3]);
    void EKF_prediction(); //[Adriano]
    void EKF_update(float z_pos[3], float z_quat[4]); //[Adriano]
    void EKF_getStates(float states[9]); //[Adriano]
    void quat2eul(float q_in[4], float ang_out[3]); //[Adriano]
    void eul2quat(float ang_in[3], float quat_out[4]); //[Adriano]
    void acrorate_control(float tau_ref, float omega_ref[3], float dt_in, float u_out[4]); //[Adriano]
    void quat_normalize(float q_in[4], float q_out[4]);
    void quat_multiply(float qa[4], float qb[4], float q_out[4]); //[Adriano]
    void quat_vec_transform(float q_in[4], float v_in[3], float v_out[3]); //[Adriano]
    void cross(float qa[3], float qb[3], float qc[3]); //[Adriano]
//    void set_motors(float u[4]); //Adriano
    void IMU_update(); //Adriano
    void compute_error_derivative(float ew[3], float ew_deriv[3]); //Adriano
    
    void normalize(float output[3], int16_t input[3]);
    void setZero();
    void headingMag(float attitude_rate[3], float  input[3], float degree[4], float throttle );
    int32_t SUM_GYRO[3] = {0}, MEAN_GYRO[3] = {0};
    int16_t accel[3] = {0}, gyro_int[3] = {0} , mag[3] = {0}, mag_int[3] = {0};
    uint8_t Buf[14];
    int16_t magOffset[3] = {0};
    int16_t mag_scale[3] = {0};
    uint8_t Mscale = MFS_16BITS; // Choose either 14-bit or 16-bit magnetometer resolution
    uint8_t Gscale = GFS_250DPS;
    uint8_t Ascale = AFS_2G;

    uint8_t Mmode = 0x06;        // 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
    
    int16_t magOffset3_ , magOffset3;
    int16_t magxOffsetMotor = 0, magyOffsetMotor  = 0, magzOffsetMotor  = 0;
    float magCaCoefficient;
    float throttle;
    float magDegree[4] = {0};
    int calAHRS;
    float headingDegreesDefoult = 0;

    int firstStage = 0;
    int magCalStage = 0;
    float magOffsetMotorCount = 0;
    float magOffsetMotor[3] = {0};
    float sumMagOffsetMotor[3] = {0};

    float degree_[4] = {1, 1, 1, 1};

    float magOffsetMotorCount2 = 0;
    float magOffsetMotor2[3] = {0};
    float sumMagOffsetMotor2[3] = {0};

    int16_t magOffsetMotorFnl[3] = {0};
    int16_t magOffsetMotorFnl2[3] = {0};

    int16_t magOffsetMotorEpr[2] = {0};
    float accel_angle[3] = {0}, angle_acc[3] = {0};

    float gyro[3] = {0}, degree[3] = {0};
    float accel_si[3] = {0}; //[Adriano] acceleration in m/s2

    float destination[3] = {1, 1, 1};
    float destination2[3] = {1, 1, 1};

    //float EKF_states[15] = {0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0}; //[x,y,z, vx,vy,vz, phi,theta,psi, bax,bay,baz, bgx,bgy,bgz]
    float EKF_states[16] = {0,0,0, 0,0,0, 0,0,0,0, 0,0,0, 0,0,0}; //[x,y,z, vx,vy,vz, qw,qx,qy,qz, bax,bay,baz, bgx,bgy,bgz]
    float u_motors[4] = {0, 0, 0, 0}; //Adriano
    float acrorateref[4] = {0, 0, 0, 0}; //Adriano
    float acrorateref_last[4] = {0, 0, 0, 0}; //Adriano
    float omega_ref_dot[3] = {0,0,0};

//    //Linear
//    float Kpwx = 15;
//    float Kpwy = 15;
//    float Kpwz = 15; //good
//    float Kiwx = 0;
//    float Kiwy = 0;
//    float Kiwz = 0; //good
//    float Kdwx = 1.4;
//    float Kdwy = 1.4;
//    float Kdwz = 0.5;
    
//    //Afine
//    float Kpwx = 7;
//    float Kpwy = 7;
//    float Kpwz = 7; //good
//    float Kiwx = 0;
//    float Kiwy = 0;
//    float Kiwz = 0; //good
//    float Kdwx = 0.7;
//    float Kdwy = 0.7;
//    float Kdwz = 0.25;

    //Afine - with w ref dot ff
    float Kpwx = 10.8;//6 + 1 + 3;
    float Kpwy = 10.8;//6 + 1;
    float Kpwz = 10;//6 + 1;
    //
    float Kiwx = 3.6;//2.5 + 1 +2;
    float Kiwy = 3.6;//2.5 + 1;
    float Kiwz = 1.8;//2.5 + 1;
    //
    float Kdwx = 4.4;//2.2 + 0.5 + 1 + 1;
    float Kdwy = 4.4;//2.2 + 0.5 + 1 + 1;
    float Kdwz = 2.5;//0.25*2 + 1;



    float EKF_gyro[3] = {0,0,0}; //gyro filtered?
    float EKF_acc[3] = {0,0,0}; //gyro filtered?
    float int_err_omega[3] = {0, 0, 0}; //Adriano
    float last_err_omega[3] = {0, 0, 0}; //Adriano


  private:

    //[Adriano]
//    //float EKF_states[15] = {0,0,0, 0,0,0, 0,0,0, 0,0,0, 0,0,0}; //[x,y,z, vx,vy,vz, phi,theta,psi, bax,bay,baz, bgx,bgy,bgz]
//    float EKF_states[16] = {0,0,0, 0,0,0, 0,0,0,0, 0,0,0, 0,0,0}; //[x,y,z, vx,vy,vz, qw,qx,qy,qz, bax,bay,baz, bgx,bgy,bgz]

    

    int16_t Filter_P[3] = {0}, Filter_I[3] = {0}, Filter_SUM[3] = {0}, delta = 0, _angle[3] = {0};
    float _rate[3] = {0}, test = 0, rate[3] = {0};
    float _w = 0, ic = 0.001, I = 0;
    float dt = 0, timeprev = 0, timenow = 0;
    int i, j = 0;
    
    float gyro_angle_x, gyro_angle_y, gyro_angle_z; // Estimated Gravity Vector
    int16_t norm = 0;
    double attitude[3] = {0}, _attitude[3]  = {0};
    uint8_t ST1;

    float avg_delta, scale_x, scale_y;

    unsigned long headingPreviousMillis = 0;
    boolean takeTimeHeading = true;
    enum Ascale {
      AFS_2G = 0,
      AFS_4G,
      AFS_8G,
      AFS_16G
    };

    enum Gscale {
      GFS_250DPS = 0,
      GFS_500DPS,
      GFS_1000DPS,
      GFS_2000DPS
    };

    enum Mscale {
      MFS_14BITS = 0, // 0.6 mG per LSB
      MFS_16BITS      // 0.15 mG per LSB
    };

    float previousHeadingDegrees = 0;
    float  headingDegrees = 0;
    int HeadingLoop = 0;
    boolean HeadingCount = false;

    int errorDirection = 0;

#define blueLed 16
#define redLed 2
#define greenLed 0

};
