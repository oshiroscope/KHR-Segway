/*
 * @project : KHR-Segway 
 * @author  : Shiro
 * @mail    : whitecastle4646@gmail.com
 */
 
//! standard GPIO
#include <Wire.h>
//! I2C device
#include <I2Cdev.h>

//! MPU6050 : IMU sensor
#include <MPU6050.h>
//! Pololu Motor Driver Shield
#include <DualVNH5019MotorShield.h>

//! power max
#define POWER_MAX 250
//! power min
#define POWER_MIN -250
//! CYCLE_TIME
#define CYCLE_TIME 15
//! power offset
#define POWER_OFFSET 5
//! theta offset
#define THETA_OFFSET (0.03)
//! L R ratio
#define LR_RATIO 1.2f
//! L R sgn
#define LR_SGN (-1)
//! omega raw val scale
#define OMEGA_SCALE 0.00013315805f

//! operate straight offset gain
#define OPERATE_STRAIGHT_GAIN 50
//! operate rotate offset gain
#define OPERATE_ROTATE_GAIN 10

//! IMU instance
MPU6050 mpu;
//! MPU vals
int16_t ax, ay, az;
int16_t gx, gy, gz;

//! Motor Driver instance
DualVNH5019MotorShield md;

//! current omega value
double omega = 0.0;
//! current theta value
double theta = 0.0;
//! sum power
int32_t sumPower;
//! sum sum power
int32_t sumSumPower;

//! theta gain (p)
const float kTheta = -1900;//-40;
//! omega gain (d)
const float kOmega = -50;//-25;
//! x gain (p)
const float kDistance = 0.0;//-25;
//! v gain (d)
const float kSpeed = -0.0;//-25;

//! power
int16_t power;
//! left input power
int16_t left_input;
//! right input power
int16_t right_input;
//! left input power offset
int16_t left_offset;
//! right input power offset
int16_t right_offset;
//! position
int32_t xE5;
//! velocity
int32_t vE5;
//! loop count
int32_t loop_count = 0;

//! emergency motor stop 
void stopIfFault(){
    if(md.getM1Fault()){
        Serial.println("M1-Fault");
        while(1);
    }
    if(md.getM2Fault()){
        Serial.println("M2-Fault");
        while(1);
    }
}

//! refresh MPU data
void renewMPU(){
    //mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz); //!遅い
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
}

//! initialization
void setup(){
    //! join I2C bus
    Wire.begin();

    //! initialize serial comm
    Serial.begin(38400);

    //! initialize device
    Serial.println("Initializing devices...");
    mpu.initialize();
    md.init();
    
    //! verify comm
    Serial.println("Testing device connections...");
    Serial.println(mpu.testConnection() ? "MPU9150 connection successful" : "MPU9150 connection failed");
}



//! loop
void loop(){
    if(millis() > loop_count * CYCLE_TIME){
        loop_count++;
        //Serial.println(millis());
        //Serial.println(loop_count);
        
        ctrl();
        Serial.print(omega,6);Serial.print("\t");
        Serial.print(theta + THETA_OFFSET,6);Serial.print("\t");
        //Serial.print(vE5);Serial.print("\t");
        //Serial.print(xE5);Serial.print("\t");
        //Serial.print(kTheta * theta);Serial.print("\t");
        //Serial.print(kOmega * omega);Serial.print("\t");
        //Serial.print(kSpeed * vE5);Serial.print("\t");
        //Serial.print(kDistance * xE5);Serial.print("\t");
        Serial.print(power);Serial.print("\t");
        Serial.print("\n");
        if(power > 0){
            left_input = power + POWER_OFFSET;
        }else if(power < 0){
            left_input = power - POWER_OFFSET;
        }else{
            left_input = 0;
        }

        right_input = left_input * LR_RATIO * LR_SGN;
        
        if(Serial.available()){
            char input = Serial.read();
            if (input == 'w'){
                left_offset = -OPERATE_STRAIGHT_GAIN;
                right_offset = OPERATE_STRAIGHT_GAIN;
            }else if(input == 'a'){
                left_offset = -OPERATE_ROTATE_GAIN;
                right_offset = -OPERATE_ROTATE_GAIN;
            }else if(input == 's'){
                left_offset = OPERATE_STRAIGHT_GAIN;
                right_offset = -OPERATE_STRAIGHT_GAIN;
            }else if(input == 'd'){
                left_offset = OPERATE_ROTATE_GAIN;
                right_offset = OPERATE_ROTATE_GAIN;
            }else{
                left_offset = 0;
                right_offset = 0;
            }
        }
        
        md.setSpeeds(left_input + left_offset, right_input + right_offset);
        stopIfFault();
    }
}


//! ctrl func
void ctrl(){
    //! data refresh
    renewMPU();    

    //! set omega
    omega = (double)gx * OMEGA_SCALE;
    //! calc theta
    double theta_acc = atan2(-(double)ay, -(double)az);
    theta = 0.98 * (theta + omega * 0.001 * CYCLE_TIME) + 0.02 * theta_acc;

    //! Ctrl
    power =
        (kTheta * (theta + THETA_OFFSET))
        +(kOmega * omega)
        +(kSpeed * vE5)
        +(kDistance * xE5);

    //! round
    power = max(min(power, POWER_MAX), POWER_MIN);

    int16_t filtered_power = power;
    //! filter power
    if(abs(power) < 20)
        filtered_power = 0;
    else if(power > 0)
        filtered_power -= 20;
    else
        filtered_power += 20;
        
    //! sum
    sumPower = sumPower + filtered_power;
    //! sum sum
    sumSumPower = sumSumPower + sumPower;

    //! velocity
    vE5 = sumPower;
    //! position
    xE5 = sumSumPower;
}

