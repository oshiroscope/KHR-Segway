/*
 * @project : KHR-Segway
 * @author  : Shiro
 * @mail    : whitecastle4646@gmail.com
 */


//#include <MsTimer2.h>
#include <FlexiTimer2.h>

//! standard GPIO
#include <Wire.h>
//! I2C device
#include <I2Cdev.h>

//! MPU6050 : IMU sensor
#include <MPU6050.h>
//! Pololu Motor Driver Shield
#include <DualVNH5019MotorShield.h>

#define DEBUG_PRINT 

//! power max
#define POWER_MAX 250
//! power min
#define POWER_MIN -250
//! CYCLE_TIME
#define CYCLE_TIME 20
//! power offset
#define POWER_OFFSET 20//5
//! theta offset
#define THETA_OFFSET (0.04)
//! L R ratio
#define LR_RATIO 1.1f
//! L R sgn
#define LR_SGN (-1)
//! omega raw val scale
#define OMEGA_SCALE 0.00013315805f

//! operate strai ght offset gain
#define OPERATE_FORWARD_GAIN 20
#define OPERATE_BACKWARD_GAIN 20
//! operate rotate offset gain
#define OPERATE_ROTATE_GAIN 10

#define W_PIN 14
#define A_PIN 15
#define S_PIN 16
#define D_PIN 17

#define LEFT_ENC_A_PIN 5
#define LEFT_ENC_B_PIN 3
#define RIGHT_ENC_A_PIN 11
#define RIGHT_ENC_B_PIN 13

#define LEFT_ENC_STEP 0.65502 // 41.7 * pi / 100 / 2
#define RIGHT_ENC_STEP 0.57648 // 36.7 * pi / 100 / 2

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
const float kTheta = -1100;//-800;//-1900;//-40;
//! omega gain (d)
const float kOmega = -20;//15;//15;//-25;
//! x gain (p)
const float kDistance = 0.5;//-25;
//! v gain (d)
const float kSpeed = 0.38;//-25;

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


//! loop count
int32_t loop_count = 0;

int16_t theta_init = 0.00f;

int16_t oldInput = 0;

//! position
volatile float linearX = 0;
volatile float rotateX = 0;
float oldLinearX;
float originLinearX = 0;

//! velocity
float linearV;

//! emergency motor stop
void stopIfFault() {
    if (md.getM1Fault()) {
        Serial.println("M1-Fault");
        //while(1);
    }
    if (md.getM2Fault()) {
        Serial.println("M2-Fault");
        //while(1);
    }
}

//! refresh MPU data
void renewMPU() {
    //mpu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz); //!遅い
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
}

//! initialization
void setup() {
    //! join I2C bus
    Wire.begin();

    //! initialize serial comm
    Serial.begin(38400);

    //! initialize device
    //Serial.println("Initializing devices...");
    mpu.initialize();
    md.init();

    //! button init
    pinMode(W_PIN, OUTPUT);
    digitalWrite(W_PIN, HIGH);
    pinMode(A_PIN, OUTPUT);
    digitalWrite(A_PIN, HIGH);
    pinMode(S_PIN, OUTPUT);

    digitalWrite(S_PIN, HIGH);
    pinMode(D_PIN, OUTPUT);
    digitalWrite(D_PIN, HIGH);

    //! enc init
    pinMode(LEFT_ENC_A_PIN, INPUT_PULLUP);
    pinMode(LEFT_ENC_B_PIN, INPUT_PULLUP);

    //! verify comm
    //Serial.println("Testing device connections...");
    //Serial.println(mpu.testConnection() ? "MPU9150 connection successful" : "MPU9150 connection failed");

    FlexiTimer2::set(0.000001, timerIRQ);
    FlexiTimer2::start();
}
unsigned long start;
//! loop
void loop() {

    if (millis() > loop_count * CYCLE_TIME) {

  
        loop_count++;
        ctrl();

#ifdef DEBUG_PRINT
        Serial.print(omega, 6); Serial.print("\t");
        Serial.print(theta + THETA_OFFSET, 6); Serial.print("\t");
        Serial.print(power); Serial.print("\t");
        //Serial.print(linearX * 1.5); Serial.print("\t"); // 1.5 mm / step
        Serial.print(linearX); Serial.print("\t"); 
        Serial.print(rotateX); Serial.print("\t"); 
        Serial.print(linearV * 1.5); Serial.print("\t");
#endif
    
        if (power > 0) {
            left_input = power + POWER_OFFSET;
        } else if (power < 0) {
            left_input = power - POWER_OFFSET;
        } else {
            left_input = 0;
        }
        right_input = left_input * LR_RATIO * LR_SGN;

        if (digitalRead(W_PIN) == LOW) {
            //left_offset = -OPERATE_FORWARD_GAIN;
            //right_offset = OPERATE_FORWARD_GAIN;
            //theta_init = 0.02f;
            originLinearX += 0.5f;
        } else if (digitalRead(A_PIN) == LOW) {
            left_offset = -OPERATE_ROTATE_GAIN;
            right_offset = -OPERATE_ROTATE_GAIN;
            theta_init = 0.00f;
        } else if (digitalRead(S_PIN) == LOW) {
            //left_offset = OPERATE_BACKWARD_GAIN;
            //right_offset = -OPERATE_BACKWARD_GAIN;
            //theta_init = - 0.02f;
            originLinearX -= 0.5f;
        } else if (digitalRead(D_PIN) == LOW) {
            left_offset = OPERATE_ROTATE_GAIN;
            right_offset = OPERATE_ROTATE_GAIN;
            theta_init = 0.00f;
        } else {
            theta_init = 0.00f;
            left_offset = 0;
            right_offset = 0;
        }

#ifdef DEBUG_PRINT
        Serial.print("\n");
#endif
    
        if (left_input > oldInput + 30)
            left_input = oldInput + 30;
        if (left_input < oldInput - 30)
            left_input = oldInput - 30;
        md.setSpeeds(left_input + left_offset, right_input + right_offset);

        oldInput = left_input;
        stopIfFault();

        //Serial.print(micros());  Serial.print("\t");
        //Serial.print(start);  Serial.print("\t");
        //Serial.println(micros() - start);
    }
}


//! ctrl func
void ctrl() {
    //! data refresh
    if (mpu.getIntDataReadyStatus()) {
        start = micros();
        renewMPU();

        //! set omega
        omega = (double)gx * OMEGA_SCALE;
        //! calc theta
        double theta_acc = atan2(-(double)ay, -(double)az);
        theta = 0.98 * (theta + omega * 0.001 * CYCLE_TIME) + 0.02 * theta_acc;

        linearV = (linearX - oldLinearX) * 1000 / CYCLE_TIME;
        oldLinearX = linearX;

        //! Ctrl
        power =
            (kTheta * (theta + THETA_OFFSET - theta_init))
            + (kOmega * omega)
            + (kSpeed * linearV);
            //+ (kDistance * (X - originLinearX));

        //! round
        power = max(min(power, POWER_MAX), POWER_MIN);

        int16_t filtered_power = power;
        const int16_t power_limit = 3;
        //! filter power
        if (abs(power) < power_limit)
            filtered_power = 0;
        else if (power > 0)
            filtered_power -= power_limit;
        else
            filtered_power += power_limit;

        //! sum
        sumPower = sumPower + filtered_power;
        //! sum sum
        sumSumPower = sumSumPower + sumPower;

    }
    else {
        Serial.println("###############");

    }
}

void timerIRQ() {
    static byte lbp = 0;
    lbp = lbp << 1;
    if (digitalRead(LEFT_ENC_A_PIN) == HIGH) {
        lbp |= 0x01;
    }
    lbp = lbp << 1;
    if (digitalRead(LEFT_ENC_B_PIN) == HIGH) {
        lbp |= 0x01;
    }

    lbp = lbp & 0x0F;

    if (lbp == 0b0111) {
	linearX += LEFT_ENC_STEP;
	rotateX -= LEFT_ENC_STEP;
    }
    if (lbp == 0b1011) {
        linearX -= LEFT_ENC_STEP;
	rotateX += LEFT_ENC_STEP;
    }

    static byte rbp = 0;
    rbp = rbp << 1;
    if (digitalRead(RIGHT_ENC_A_PIN) == HIGH) {
        rbp |= 0x01;
    }
    rbp = rbp << 1;
    if (digitalRead(RIGHT_ENC_B_PIN) == HIGH) {
        rbp |= 0x01;
    }

    rbp = rbp & 0x0F;

    if (rbp == 0b0111) {
	linearX -= RIGHT_ENC_STEP;
	rotateX -= RIGHT_ENC_STEP;
    }
    if (rbp == 0b1011) {
        linearX += RIGHT_ENC_STEP;
	rotateX += RIGHT_ENC_STEP;
    }

}

