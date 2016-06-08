/*

  ***************************************
  ******* Self balancing unicycle *******
  ***************************************
  
  Need to download these libs in order to work:
  i2c -> http://diyhacking.com/projects/I2Cdev.zip


  ***** Connections *****

  * MPU6050  UNO/NANO
  * VCC      +5v
  * GND      GND
  * SCL      A5
  * SDA      A4
  * Int	     D2
  
  * Motor Driver BTS7960
  * VCC   +5v
  * GND   GND
  * RPWM    D5      Forward pwm input
  * LPWM    D6      Reverse pwm input
  * R_EN    D7      Forward drive enable input, can be bridged with L_EN
  * L_EN    D8      Reverse drive enable input, can be bridged with R_EN
  * R_IS	-	Current alarm, not used
  * L_IS	-	Current alarm, not used
  * B+    Battery+
  * B-    Battery-
  * M+    Motor+
  * M-    Motor-
  
  * Voltage divider 0-24v -> 0-5v ( R1: 380k, R2: 100k)
  * Vout    A3

  ***** Connections *****


  Credits MCU part: http://www.pitt.edu/~mpd41/Angle.ino

*/

#include <Wire.h>

// PID constants
float kp = 0.8; //Value between 0 <-> 0.8
float ki = 1.0; 
float kd = 0.5; //Value between 0 <-> 0.5

bool ki_enable = FALSE; //Enable integral regulator if true, disable if false

float setpoint = 0.0; // Initial setpoint

int deadband = 2; // +-degrees of deadband around setpoint
int max_pitch = 15; // Degrees before motor cut *SAFETY FUNCTION*
int max_roll = 15; // Degrees before motor cut *SAFETY FUNCTION*

// PID variables
int Umax = 255;  // Max output
int Umin = -255; // Min output

float p_term = 0.0; // Store propotional value
float i_term = 0.0; // Store integral value
float d_term = 0.0; // Store derivative value

float error = 0.0; // Sum error
float last_error = 0.0; // Store last error sum

int output = 0.0; // PID output

// Timers
int main_loop_timer = millis();
int lastTime = millis();

// Input voltage divider
int battery_voltage_input = A3;

// Motor Driver BTS7960 pins
int RPWM = 5; // Forward pwm input
int LPWM = 6; // Reverse pwm input
int R_EN = 7; // Forward drive enable input
int L_EN = 8; // Reverse drive enable input

// MCU variables
float degconvert = 57.2957786; // There are 57 degrees in a radian
float dt = (10.0/1000.0); // 100hz = 10ms
int MPU_addr = 0x68;
int mcu_prev_dt; // MCU previous loop time
float AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; // The raw data from the MPU6050.
float roll = 0.0;
float pitch = 0.0;
float gyroXangle = 0.0;
float gyroYangle = 0.0;
float gyroXrate = 0.0;
float gyroYrate = 0.0;
float filtered_angle_pitch = 0.0;
float filtered_angle_roll = 0.0;
float mcu_dt = 0.0; // MCU loop time



// ***** PID function *****
float get_pid(float measured_angle)
{
  float delta_t = millis() - lastTime; // Delta time
  lastTime = millis(); // Reset timer

  error = setpoint - measured_angle; // Calculate error, e=SP-Y

  p_term = kp * error;  // Propotional
  
  if (ki_enable == TRUE) {  // Integral
  	i_term += (ki * error * delta_t);
  }
  else i_term = 0; //Disabled integral regulator
  
  d_term = kd * ((error - last_error) / delta_t); // Derivative

  output = p_term + i_term + d_term; // Calculate output

  // Limit output
  if (output > Umax) output = Umax;
  else if (output < Umin) output = Umin;
  
  last_error = error; // Remember error for next time

  return output;
}



// ***** Motor output *****
void motor(int pwm)
{
  // Set direction
  if (pwm > (0 + deadband))
  { 
    // Drive forward
    analogWrite(RPWM, abs(pwm)); 
  }
  else if (pwm < (0 - deadband))
  { 
    // Drive backward
    analogWrite(LPWM, abs(pwm)); 
  }
  else
  { 
    // Stop motor
    digitalWrite(R_EN, LOW);
    digitalWrite(L_EN, LOW);
    digitalWrite(RPWM, LOW);
    digitalWrite(LPWM, LOW);
  }
}



// ***** Get angle from MPU6050 gyro/acc *****
void get_angle() 
{ 
  // Collect raw data from the sensor.
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  mcu_dt = (float)(millis() - mcu_prev_dt) * 1000; //This line does three things: 1) stops the timer, 2)converts the timer's output to seconds from microseconds, 3)casts the value as a double saved to "dt".
  mcu_prev_dt = millis(); //start the timer again so that we can calculate the next dt.

  //the next two lines calculate the orientation of the accelerometer relative to the earth and convert the output of atan2 from radians to degrees
  //We will use this data to correct any cumulative errors in the orientation that the gyroscope develops.
  roll = atan2(AcY, AcZ)*degconvert;
  pitch = atan2(-AcX, AcZ)*degconvert;

  //The gyroscope outputs angular velocities.  To convert these velocities from the raw data to deg/second, divide by 131.  
  //Notice, we're dividing by a double "131.0" instead of the int 131.
  gyroXrate = GyX/131.0;
  gyroYrate = GyY/131.0;
}



// ***** Main setup *****
void setup()
{ 
  // ***** MPU6050 setup *****
  Wire.begin();
  #if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
  #else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
  #endif

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  delay(100);

  // setup starting angle
  //1) collect the data
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  //2) calculate pitch and roll
  roll = atan2(AcY, AcZ)*degconvert;
  pitch = atan2(-AcX, AcZ)*degconvert;

  //3) set the starting angle to this pitch and roll
  gyroXangle = roll;
  gyroYangle = pitch;
  filtered_angle_roll = roll;
  filtered_angle_pitch = pitch;


  // ***** Read battery voltage *****
  pinMode(battery_voltage_input, INPUT);


  // ***** Motorcontroller setup *****
  pinMode(RPWM, OUTPUT); // PWM output right channel
  pinMode(LPWM, OUTPUT); // PWM output left channel
  pinMode(R_EN, OUTPUT); // Enable right channel
  pinMode(L_EN, OUTPUT); // Enable left channel

  digitalWrite(RPWM, LOW); // Disable motor @ start
  digitalWrite(LPWM, LOW);
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);



  // ***** Begin serial port *****
  Serial.begin(115200);



  // ***** Initialize timer *****
  mcu_prev_dt = millis(); // Initialize MCU timer
  main_loop_timer = millis(); // Initialize main loop timer
}



// ***** Main loop *****
void loop()
{ 
  if ((millis() - main_loop_timer) > (dt * 1000)) // Run loop @ 100hz (1/100hz = 10ms)
  {
    main_loop_timer = millis(); // Reset main loop timer
    
    // **** Read battery voltage input and convert to 0-100% ****
	int battery_voltage = map(analogRead(battery_voltage_input), 0, 1023, 0, 100);

    get_angle(); //Get angle from MCU6050

    // Calculate the angle using a Complimentary filter
    filtered_angle_roll = 0.98 * (filtered_angle_roll + gyroXrate * mcu_dt) + 0.02 * roll;  // X-axis
	filtered_angle_pitch = 0.98 * (filtered_angle_pitch + gyroYrate * mcu_dt) + 0.02 * pitch; // Y-axis

    //Calculate PID output
    int pid_output = get_pid(filtered_angle_pitch); // +-255

	// If xy angle is greater than max degrees, stop motor *SAFETY*
	if ((abs(filtered_angle_pitch) > max_pitch) && (abs(filtered_angle_roll) > max_roll))
	{
		digitalWrite(R_EN,LOW); // Stop motor
		digitalWrite(L_EN,LOW);
		motor(0);
    }
    else
    { 
		digitalWrite(R_EN,HIGH); // Enable and write PID output to motor
		digitalWrite(L_EN,HIGH);
		motor(pid_output);
    }
  }
}
