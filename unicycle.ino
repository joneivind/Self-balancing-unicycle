/*

  ***************************************
  ******* Self balancing unicycle *******
  ***************************************

  Part of final year project in electrical engineering @ Høyskolen i Oslo Akershus, Norway.
  By Jon-Eivind Stranden & Nicholai Kaas Iversen © 2016.
  https://github.com/joneivind
  

  ***** Connections *****
  
  * SENSOR   UNO/NANO   MEGA2560
  
  * MPU6050 gyro/acc (I2C)
  * VCC      +5v        +5v
  * GND      GND        GND
  * SCL      A5         C21
  * SDA      A4         C20
  * Int      D2         D2      (Optional)
  
  * Motor Driver BTS7960
  * VCC     +5v
  * GND     GND
  * RPWM    D9          Forward pwm input
  * LPWM    D10         Reverse pwm input
  * R_EN    D7          Forward drive enable input, can be bridged with L_EN
  * L_EN    D8          Reverse drive enable input, can be bridged with R_EN
  * R_IS    -           Current alarm, not used
  * L_IS    -           Current alarm, not used
  * B+      Battery+
  * B-      Battery-
  * M+      Motor+
  * M-      Motor-
  
  * 16x2 LCD display with I2C backpack
  * VCC     +5v         +5v
  * GND     GND         GND
  * SCL     A5          C21
  * SDA     A4          C20
  
  * Reset button
  * SIGNAL  D5          D5
  * LED     D6          D6
  * GND     GND         GND
  
  * Voltage divider 0-24v -> 0-5v ( R1: 380k, R2: 100k)
  * Vout    A3          A3    Vout + from battery

  ***** Connections end *****

  Credits:
  MPU6050 part: http://www.pitt.edu/~mpd41/Angle.ino
  PWM frequency change: http://coolkidsrobots.com/guide/changing-pwm-frequencies-arduino

*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x3F,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

// PID constants
float kp = 45.0;
float ki = 0.0005; 
float kd = 16.0;

// PID output variable
float kp_1;
float kd_1;

float setpoint = 84.0; // Initial degree setpoint

bool ki_enable = false; //Enables integral regulator if true, disabled if false

int deadband = 0; // +-degrees of deadband around setpoint where motor output is zero
int max_roll = 12; // Degrees from setpoint before motor will stop
int min_roll = 8; // Degrees from setpoint before motor will stop

//Pushback function
float pushback_angle = 8.0;  // Degrees where pushback should activate *Must be less than max_roll*
float pushback_range = 6.0;  // Degrees from setpoint where pushback deactivates if activated
float pushback_factor = 1.1;  // How much increase in PID when pushback
bool pushback_enable = false; // Default pushback_enable value *DONT CHANGE*

bool motor_direction_forward = false;  // Set motor direction forward/reverse

bool fall_detection_trigger = false; // Default value fall detection *DONT CHANGE*

// PID variables
float Umax = 150.0;  // Max output
float Umin = -150.0; // Min output

float p_term = 0.0; // Store propotional value
float i_term = 0.0; // Store integral value
float d_term = 0.0; // Store derivative value

float error = 0.0; // Sum error
float last_error = 0.0; // Store last error sum

int output = 0.0; // PID output

// Timers
int main_loop_timer = millis(); // Dt timer main loop 
int lastTime = millis(); //Dt timer PID loop

// Input voltage divider
int battery_voltage_input = A3; // Sensor value 0-1023

// Motor Driver BTS7960 pins
int RPWM = 9; // Forward pwm input
int LPWM = 10; // Reverse pwm input
int R_EN = 7; // Forward drive enable input
int L_EN = 8; // Reverse drive enable input

// Reset button
int reset_button_pin = 5;
int reset_button_led_pin = 6;
bool reset_button_led_state = true;
int led_counter = 0;

// MPU6050 (Gyro/acc) variables
const int MPU_addr=0x68;
double AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //These will be the raw data from the MPU6050.
uint32_t timer; //it's a timer, saved as a big-ass unsigned int.  We use it to save times from the "micros()" command and subtract the present time in microseconds from the time stored in timer to calculate the time for each loop.
double compAngleX, compAngleY; //These are the angles in the complementary filter
#define degconvert 57.2957786 //there are like 57 degrees in a radian.



// ***** PID function *****
float get_pid(float angle)
{
  float delta_t = millis() - lastTime; // Delta time
  lastTime = millis(); // Reset timer

  error = setpoint - angle; // Calculate error, e=SP-Y
  
  // Pushback function if near max roll
  
  if (angle > (setpoint + pushback_angle) || angle < (setpoint - pushback_angle)) {
    pushback_enable = true;
  }
  else if (pushback_enable && angle < (setpoint + pushback_range) && angle > (setpoint - pushback_range)) {
    pushback_enable = false;
  }
  /*else {
    pushback_enable = false;
  }*/
  
  if (pushback_enable) {
    kp_1 = kp * pushback_factor; // Increased kp
    kd_1 = kd * pushback_factor; // Increased kd
  }
  else {
    kp_1 = kp; // Stock kp
    kd_1 = kd; // Stock kd
  }

 // Calculate PID
  p_term = kp_1 * error;  // Propotional
  
  if (ki_enable == true) {  // Integral
    i_term += (ki * error * delta_t);
  }
  else i_term = 0; // Disable integral regulator
  
  d_term = kd_1 * ((error - last_error) / delta_t); // Derivative

  output = p_term + i_term + d_term; // Calculate output

  // Limit output
  if (output > Umax) {
    output = Umax;
  }
  else if (output < Umin) {
    output = Umin;
  }
  
  last_error = error; // Remember error for next time

  return output;
}



// ***** Motor output *****
void motor(int pwm, float angle)
{
  if (motor_direction_forward == true)
  {
    if (angle > (setpoint + deadband))
    { 
      // Drive backward
      analogWrite(LPWM, abs(pwm)); 
    }
    else if (angle < (setpoint - deadband))
    { 
      // Drive forward
      analogWrite(RPWM, abs(pwm)); 
    }
  }
  else
  {
    if (angle > (setpoint + deadband))
    { 
      // Drive backward
      analogWrite(RPWM, abs(pwm)); 
    }
    else if (angle < (setpoint - deadband))
    { 
      // Drive forward
      analogWrite(LPWM, abs(pwm)); 
    }
  }
}



// ***** Get angle from MPU6050 gyro/acc *****
float get_angle() 
{ 
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
  
  double dt = (double)(micros() - timer) / 1000000; //This line does three things: 1) stops the timer, 2)converts the timer's output to seconds from microseconds, 3)casts the value as a double saved to "dt".
  timer = micros(); //start the timer again so that we can calculate the next dt.

  //the next two lines calculate the orientation of the accelerometer relative to the earth and convert the output of atan2 from radians to degrees
  //We will use this data to correct any cumulative errors in the orientation that the gyroscope develops.
  double roll = atan2(AcY, AcZ)*degconvert;
  double pitch = atan2(-AcX, AcZ)*degconvert;

  //The gyroscope outputs angular velocities.  To convert these velocities from the raw data to deg/second, divide by 131.  
  //Notice, we're dividing by a double "131.0" instead of the int 131.
  double gyroXrate = GyX/131.0;
  double gyroYrate = GyY/131.0;

  //THE COMPLEMENTARY FILTER
  //This filter calculates the angle based MOSTLY on integrating the angular velocity to an angular displacement.
  //dt, recall, is the time between gathering data from the MPU6050.  We'll pretend that the 
  //angular velocity has remained constant over the time dt, and multiply angular velocity by 
  //time to get displacement.
  //The filter then adds a small correcting factor from the accelerometer ("roll" or "pitch"), so the gyroscope knows which way is down. 
  compAngleX = 0.98 * (compAngleX + gyroXrate * dt) + 0.02 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.98 * (compAngleY + gyroYrate * dt) + 0.02 * pitch;

  return compAngleX;
}



// ***** Read battery voltage *****
int read_voltage()
{
  // Read battery voltage input and convert to 0-24v
  int battery_voltage = map(analogRead(battery_voltage_input), 0, 1023, 0, 26);
  
  return battery_voltage;
}



// ***** Main setup *****
void setup()
{ 
  // ***** Initialize lcd display *****
  lcd.begin();
  lcd.clear(); // Clear display
  lcd.backlight(); // Turn on backlight
  lcd.setCursor(0, 0);
  lcd.print("   SB Unicycle  ");
  lcd.setCursor(0, 1);
  lcd.print("   Booting...   ");


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
  Serial.begin(115200);
  delay(100);

  //setup starting angle
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
  double roll = atan2(AcY, AcZ)*degconvert;
  double pitch = atan2(-AcX, AcZ)*degconvert;

  //3) set the starting angle to this pitch and roll
  double gyroXangle = roll;
  double gyroYangle = pitch;
  double compAngleX = roll;
  double compAngleY = pitch;
  
  
  // Change PWM frequency (Affects servo timer)
  TCCR1B = TCCR1B & B11111000 | B00000010; // set timer 1 divisor to 8 for PWM frequency of 3921.16 Hz


  //start a timer
  timer = micros();


  // ***** Battery voltage *****
  pinMode(battery_voltage_input, INPUT);


  // ***** Motorcontroller *****
  pinMode(RPWM, OUTPUT); // PWM output right channel
  pinMode(LPWM, OUTPUT); // PWM output left channel
  pinMode(R_EN, OUTPUT); // Enable right channel
  pinMode(L_EN, OUTPUT); // Enable left channel
  digitalWrite(RPWM, LOW); // Disable motor @ start
  digitalWrite(LPWM, LOW);
  digitalWrite(R_EN, LOW);
  digitalWrite(L_EN, LOW);
  
  
  // ***** Reset button *****
  pinMode(reset_button_pin, INPUT);
  pinMode(reset_button_led_pin, OUTPUT);
  

  // ***** Begin serial port *****
  Serial.begin(115200);

  
  // ***** Initialize timer *****
  //mpu_prev_dt = millis(); // Initialize MPU timer
  main_loop_timer = millis(); // Initialize main loop timer



  // Add some initial gyro angle values
  for (int i=0; i<1000; i++)
  {
    get_angle();
  }
  
  
  // Waiting for upright position
  lcd.clear();
  while(int(setpoint - get_angle()) != 0)
  {
    get_angle();
    lcd.setCursor(0, 0);
    lcd.print("Keep centered    ");
    lcd.setCursor(0, 1);
    lcd.print("Wait for zero ");
    lcd.print(int(setpoint - get_angle()));
    lcd.print("   ");
  }
  lcd.clear();
}



// ***** Main loop *****
void loop()
{ 
  if ((millis() - main_loop_timer) > (10.0/1000.0 * 1000)) // Run loop @ 100hz (1/100hz = 10ms)
  {
    main_loop_timer = millis(); // Reset main loop timer

    float angle = get_angle();
    
    
    //Calculate PID output
    int pid_output = get_pid(abs(angle)); // +-255

    // If roll angle is greater than max roll, stop motor
    if (angle > (setpoint + max_roll) || angle < (setpoint - min_roll))
    {
      digitalWrite(R_EN,LOW);
      digitalWrite(L_EN,LOW);
      motor(0, angle);
      
      fall_detection_trigger = true; // Fall detected

      if(fall_detection_trigger = true)
      {
        lcd.setCursor(0, 0);
        lcd.print(" FALL DETECTED! ");
        lcd.setCursor(0, 1);
        lcd.print(" Please reset...");
          
        while(1);
      }
    }
    
    else
    { 
      // Enable and write PID output value to motor
      digitalWrite(R_EN,HIGH);
      digitalWrite(L_EN,HIGH);
      motor(pid_output, angle);
      
      // ***** LCD output *****
        
      // Battery monitor
      lcd.setCursor(0, 0);
      lcd.print("OUT:");
      lcd.setCursor(4, 0);
      lcd.print(abs(int((100.0/Umax)*pid_output)));
      lcd.print("%  ");
      
      // Angle offset
      lcd.setCursor(0, 1);
      lcd.print("Offs:");
      lcd.setCursor(5, 1);
      lcd.print(int(setpoint - angle));
      lcd.print("  ");
      
      //PID values
      lcd.setCursor(9, 0);
      lcd.print(" P:");
      lcd.setCursor(12, 0);
      lcd.print(kp_1);
      lcd.setCursor(9, 1);
      lcd.print(" D:");
      lcd.setCursor(12, 1);
      lcd.print(kd_1);
          
      // ***** LCD output end *****

      // DEBUG Serial display
      Serial.print(setpoint - angle);
      Serial.print("\t");
      Serial.println(pid_output/10.0);
    }
  }
}
