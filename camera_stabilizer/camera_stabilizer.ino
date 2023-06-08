#include <Wire.h>
#include <AccelStepper.h>


long g_yaw_pos = 0;
long g_pitch_pos = 0;

//modified by shabir Yousofi from 50 to 5 from faster motor response -> less reading = faster motor response
int num_readings = 5;  // amount of reading the accelerometer makes
float oldPitch = 0;
float oldRoll = 0;
float x_angle = 0;
float z_angle = 0;
float threshold = 500.0f;

typedef struct YawPitchRoll_t {
  int32_t yaw;
  int32_t pitch;
  int32_t roll;
} YawPitchRoll_s;

typedef struct Acc_t {
  int32_t x;
  int32_t y;
  int32_t z;
} Acc_s;

const int MPU = 0x68;  //mpu addres

//modified by Shabir Yousofi
const int yaw_limit_sw = 11;    // arduino pin 11, CNC shield Z-/Z+
const int pitch_limit_sw = 10;  // arduino pin 10, CNC shield Y-/Y+
const int roll_limit_sw = 9;    // arduino pin 9, CNC shield X-/X+

const int microstepping_multiplier = 8;                                     // e.g. 1/16th microstepping -> set to 16
const float yaw_offset = -45.0f, pitch_offset = 0.34f, roll_offset = 1.9f;  // degree


// modified by shabir Yousofi
AccelStepper step_y(AccelStepper::DRIVER, 4, 7);  // yaw  = DirZ
AccelStepper step_p(AccelStepper::DRIVER, 3, 6);  // pitch = DirY
AccelStepper step_r(AccelStepper::DRIVER, 2, 5);  // roll = DirX




void setup() {
  Serial.begin(115200);  // set baudrate to 115200
  Serial.setTimeout(1);

  for (int i = 2; i <= 8; i++) {  // set driver step/dir pins and drive enable pin to output
    pinMode(i, OUTPUT);
  }
  digitalWrite(8, LOW);  // drive enabled, active at low level

  // Set limit switch pins as inputs with internal pull-up resistors
  pinMode(yaw_limit_sw, INPUT_PULLUP);
  pinMode(pitch_limit_sw, INPUT_PULLUP);
  pinMode(roll_limit_sw, INPUT_PULLUP);

  //Initialisation of Steppemotors 17HS4023
  step_y.setMaxSpeed(2000);
  step_y.setAcceleration(1000);
  step_y.setPinsInverted(true, false, false);
  step_y.setCurrentPosition(0);

  // modified by shabir Yousofi
  step_p.setMaxSpeed(8000);
  step_p.setAcceleration(8000);
  step_p.setPinsInverted(true, false, false);  // direction inverted, step normal, enable normal
  step_p.setCurrentPosition(0);

  // modified by shabir Yousofi
  step_r.setMaxSpeed(8000);
  step_r.setAcceleration(8000);
  step_r.setPinsInverted(true, false, false);  // direction inverted, step normal, enable normal
  step_r.setCurrentPosition(0);
  //step_p.setMinPulseWidth(20);

  Wire.begin();
  Wire.setClock(400000);
  setup_mpu_6050();
  autohome();
}

void autohome() {

  //Modified by Shabir Yousofi
  step_p.enableOutputs();  // Calibrate pitch
  step_p.setSpeed(-600 * microstepping_multiplier);
  while (digitalRead(pitch_limit_sw) == HIGH) {
    step_p.runSpeed();
  }
  step_p.setCurrentPosition(-600 * microstepping_multiplier);
  step_p.moveTo(0);
  while (step_p.run()) {
    // block while moving
  }

  //Modified by Shabir Yousofi
  step_r.enableOutputs();  // Calibrate roll
  step_r.setSpeed(-600 * microstepping_multiplier);
  while (digitalRead(roll_limit_sw) == HIGH) {
    step_r.runSpeed();
  }
  step_r.setCurrentPosition(-600 * microstepping_multiplier);
  step_r.moveTo(0);
  while (step_r.run()) {
    // block while moving
  }
}

void readAngles() {
  String input = Serial.readStringUntil('\n');

  run_steppers();

  // Slice received string into x,z,threshold
  String x_string = input.substring(0, input.indexOf(':'));
  String z_string = input.substring(input.indexOf(':') + 1, input.indexOf(';'));  //input.indexOf('\n')
  String user_input = input.substring(input.indexOf(';') + 1, input.indexOf('\n'));

  run_steppers();

  //modiefied by shabir Yousofi
  if (user_input != "n") {
    threshold = user_input.toFloat();  //threshold
  }

  char x_array[x_string.length() + 1];
  char z_array[z_string.length() + 1];

  run_steppers();
  x_string.toCharArray(x_array, x_string.length() + 1);
  z_string.toCharArray(z_array, z_string.length() + 1);

  run_steppers();

  // Constrains for the received angles so they can only make real turns
  float x = atof(x_array);
  if (x <= 380) {
    x_angle = x;
  }

  float z = atof(z_array);
  if (z <= 45) {
    z_angle = z;
  }
}

void loop() {
  while (Serial.available() > 0) {
    // Since this loop can take quite long to ensure stability and smoothness in the video
    // the steppers should check to run the steppers as often as possible. This is repeated
    // throughout this loop.
    run_steppers();
    readAngles();
    run_steppers();

    // Calculate angle to steps for the motor
    g_yaw_pos = ((x_angle * microstepping_multiplier) / 1.8) * 1.35;   //1.35 is just a ratio otherwise the system wont fully rotate.
    g_pitch_pos = ((-z_angle * microstepping_multiplier) / 1.8) * 20;  // 20 is just a ratio higher number = longer rotation.

    run_steppers();
  }

  //read accelerometer x times and calculate average
  static Acc_s acc = read_mpu_6050_data();
  Acc_s tmp_acc = { .x = 0, .y = 0, .z = 0 };
  for (int i = 0; i < num_readings; i++) {
    Acc_s reading = read_mpu_6050_data();
    tmp_acc.x += reading.x;
    tmp_acc.y += reading.y;
    tmp_acc.z += reading.z;
    delayMicroseconds(threshold);  //changing this value changes the speed of stabilisation (lower = faster but less accurate)
    run_steppers();
  }

  acc.x = (tmp_acc.x / num_readings);
  acc.y = (tmp_acc.y / num_readings);
  acc.z = (tmp_acc.z / num_readings);

  run_steppers();

  YawPitchRoll_s ypr = calculate_pitch_roll_from_acc(acc);

  run_steppers();

  ypr.yaw += yaw_offset;
  ypr.pitch += pitch_offset;
  ypr.roll += roll_offset;
  ypr.yaw = constrain(ypr.yaw, -15.0f, 15.0f);
  ypr.pitch = constrain(ypr.pitch, -15.0f, 15.0f);
  ypr.roll = constrain(ypr.roll, -15.0f, 15.0f);

  run_steppers();

  //Move motors to calulated positions
  step_y.moveTo(g_yaw_pos);

  //modified by Shabir Yousofi
  int step_p_setpoint = (ypr.pitch * (32000 * 200 * microstepping_multiplier / 360)) * 5;  // <gear ratio> * <steps per stepper revolution> * microstepping / 360 = steps per degree
  int calcPitch = g_pitch_pos - step_p_setpoint;
  calcPitch = constrain(calcPitch, -700000, 700000);
  step_p.moveTo(-calcPitch);

  //Modified by shabir Yousofi
  int step_r_setpoint = (ypr.roll * (32000 * 200 * microstepping_multiplier / 360)) * 5;
  step_r_setpoint = constrain(step_r_setpoint, -700000, 700000);
  step_r.moveTo(-step_r_setpoint);

  run_steppers();
  int delay_in_ms = 25;
  static unsigned long timestamp = millis();
  while (timestamp + delay_in_ms > millis()) {
    run_steppers();
  }
  timestamp += delay_in_ms;
}

void run_steppers() {
  step_y.run();
  step_p.run();
  step_r.run();
}


// written by Gaurish Arkesteyn
void setup_mpu_6050() {
  // Activate the MPU-6050
  Wire.beginTransmission(MPU);  // Start communicating with the MPU-6050
  Wire.write(0x6B);             // Send the requested starting register
  Wire.write(0x00);             // Set the requested register
  Wire.endTransmission();
  // Configure the accelerometer (+/-2g)
  Wire.beginTransmission(MPU);  // Start communicating with the MPU-6050
  Wire.write(0x1C);             // Send the requested starting register
  Wire.write(0x00);             // Set the requested register
  Wire.endTransmission();
  // Configure the gyro (250 degrees per second)
  Wire.beginTransmission(MPU);  // Start communicating with the MPU-6050
  Wire.write(0x1B);             // Send the requested starting register
  Wire.write(0x00);             // Set the requested register
  Wire.endTransmission();
}

YawPitchRoll_s calculate_pitch_roll_from_acc(Acc_s acc) {
  YawPitchRoll_s ypr;
  //fypr.yaw = calcYaw(acc.x, acc.y, acc.z);
  ypr.pitch = PitchRoll(acc.y, acc.x, acc.z);
  ypr.roll = -PitchRoll(acc.x, acc.y, acc.z);
  return ypr;
}

// written by Gaurish Arkesteyn
// function to calculate the pitch and roll: https://engineering.stackexchange.com/questions/3348/calculating-pitch-yaw-and-roll-from-mag-acc-and-gyro-data
// Formula: pitch = 180 * atan (accelerationX/sqr(accelerationY * accelerationY + accelerationZ * accelerationZ))/PI;
// Formula: roll = 180 * atan (accelerationY/sqr(accelerationX * accelerationX + accelerationZ * accelerationZ))/PI;
float PitchRoll(float A, float B, float C) {
  float DataA, DataB, R_Value;
  DataA = A;
  DataB = sqrt((B * B) + (C * C));

  R_Value = atan2(DataA, DataB);
  R_Value = R_Value * 180 / PI;

  return R_Value;
}

//modified by shabir Yousofi --> x and y are rotated because the sensor was placed in the wrong orientation
// written by Gaurish Arkesteyn, modified by Henkjan Veldhoven
Acc_s read_mpu_6050_data() {
  Wire.beginTransmission(MPU);     // Start communicating with the MPU-6050
  Wire.write(0x3B);                // Send the requested starting register
  Wire.endTransmission();          // End the transmission
  Wire.requestFrom(MPU, 6, true);  // request a total of 14 registers
  //read from the MPU6050 registers and store the data in the predefined variables
  Acc_s acc;
  acc.y = Wire.read() << 8 | Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  acc.x = Wire.read() << 8 | Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  acc.z = Wire.read() << 8 | Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  //  temp = Wire.read() << 8 | Wire.read();   // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  //  gyro_x = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  //  gyro_y = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  //  gyro_z = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  return acc;
}
