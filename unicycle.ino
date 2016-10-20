/*

  ///////////////////////////////////////
  ******* Self balancing unicycle *******
  ///////////////////////////////////////

  Final year project electrical engineering @ Høyskolen i Oslo Akershus, Norway © 2016.
  By Jon-Eivind Stranden & Nicholai Kaas Iversen.
  
  Web: https://github.com/joneivind
  Contact: joneivinds@gmail.com

  
  ///////////////////////
  ***** Connections *****
  ///////////////////////
  
  * SENSOR   UNO/NANO   MEGA2560 (Same if not specified)
  
  ///////////////////////////
  // MPU6050 gyro/acc (I2C)//
  ///////////////////////////
  * VCC      +5v        +5v
  * GND      GND        GND
  * SCL      A5         C21
  * SDA      A4         C20
  
  //////////////////////////
  // Motor Driver BTS7960 //
  //////////////////////////
  * VCC     +5v
  * GND     GND
  * RPWM    D9                  Forward pwm input
  * LPWM    D10                 Reverse pwm input
  * R_EN    D7                  Forward drive enable input, can be bridged with L_EN
  * L_EN    D8                  Reverse drive enable input, can be bridged with R_EN
  * R_IS                        Current alarm, not used
  * L_IS                        Current alarm, not used
  * B+                          Battery+
  * B-                          Battery-
  * M+                          Motor+
  * M-                          Motor-

  ////////////////////////////////////////
  // 16x2 LCD display with I2C backpack //
  ////////////////////////////////////////
  * VCC     +5v         
  * GND     GND         
  * SCL     A5          C21
  * SDA     A4          C20
  
  //////////////
  // Ledstrip //
  //////////////
  * VCC     +5v 
  * GND     GND
  * SIGNAL  D6

  ////////////
  // Buzzer //
  ////////////
  * SIGNAL  D11
  * GND     GND

  //////////////////
  // Reset button //
  //////////////////
  * SIGNAL  D4
  * GND     GND

  ///////////////////////////////////////////////////////////////
  // Voltage divider 0-24v -> 0-5v ( R1: 470k, R2: 100k + 10k) //
  ///////////////////////////////////////////////////////////////
  * Vout    A3                  Vout + from battery


  *** Credits *** 
  MPU6050/Kalman filter: https://github.com/TKJElectronics/KalmanFilter
  Medianfilter: http://www.elcojacobs.com/eleminating-noise-from-sensor-readings-on-arduino-with-digital-filtering/

*/


////////////////////////////////////////////
// Library /////////////////////////////////
////////////////////////////////////////////


      #include <Wire.h>
      #include <Kalman.h>
      #include <LiquidCrystal_I2C.h>
      #include <Adafruit_NeoPixel.h>
      #include <avr/power.h>
      #include <PWM.h>
      
      
////////////////////////////////////////////
// PID constants ///////////////////////////
////////////////////////////////////////////


      float kp = 55.0; // P-value
      float kd = 7.0; // D-value
      float setpoint = 81.0; // Setpoint (Balance point)
      
      //#define TUNING // Uncomment if tuning panel is attached


////////////////////////////////////////////
// Output settings /////////////////////////
////////////////////////////////////////////


      float Umax = 255.0;  // Adjust max output 0-255
      float Umin = -255.0; // Adjust Min output 0-(-255)

      int max_roll = 15; // Max degrees from setpoint before motor will stop
      int min_roll = 10; // Min degrees from setpoint before motor will stop


      // Storage variables
      float p_term = 0.0; // Store propotional value
      float d_term = 0.0; // Store derivative value
      
      float error = 0.0; // Sum error
      float last_error = 0.0; // Store last error sum
      
      int output = 0; // PID output
      int total_output = 0; // PID output + throttle_expo
      int pid_output = 0; // PID out in main loop
      float angle = 0;


////////////////////////////////////////////
// Pushback function (Not active) //////////
////////////////////////////////////////////


      float pushback_angle = 2.1;  // Degrees where pushback should activate *Must be less than max_roll*
      float throttle_expo_factor = 11.0; // How strong the throttle expo is after the pushback angle
      int throttle_expo = 0; // Standard throttle_expo value *DONT CHANGE*

      
////////////////////////////////////////////
// Stats ///////////////////////////////////
////////////////////////////////////////////

      
      // Storage variables
      bool enableStats = false; // Turns on stats when angle is zero 
      int maxOutput = 0; // Shows max pid output
      int maxAngle = 0; // Shows max angle


////////////////////////////////////////////
// Timers //////////////////////////////////
////////////////////////////////////////////


      int lastTime = millis(); // Dt timer PID loop


////////////////////////////////////////////
// Motor driver settings ///////////////////
////////////////////////////////////////////

      
      bool motor_direction_forward = false;  // Set motor direction forward/reverse
      int frequency = 4000; // Default motor frequency (in Hz)
      
      int RPWM = 9; // Forward pwm input
      int LPWM = 10; // Reverse pwm input
      int R_EN = 7; // Forward drive enable input
      int L_EN = 8; // Reverse drive enable input


////////////////////////////////////////////
// Button menu /////////////////////////////
////////////////////////////////////////////


      int menu_button_pin = 4; // Menu button pin
      int menu_item = 3; // Default frequency menu item


////////////////////////////////////////////
// MPU6050 (Gyro/acc) variables ////////////
////////////////////////////////////////////

     
      #define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead of ±180deg
      
      double accX, accY, accZ; //From IMU
      double gyroX, gyroY, gyroZ; //From IMU
      int16_t tempRaw; //From IMU
      
      double angleX = 0.0;
      double angleY = 0.0;
      
      double gyroXangle, gyroYangle; // Angle calculate using the gyro only
      double compAngleX, compAngleY; // Calculated angle using a complementary filter
      double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
      
      uint32_t timer;
      uint8_t i2cData[14]; // Buffer for I2C data

      Kalman kalmanX; // Create the Kalman instances
      Kalman kalmanY;


////////////////////////////////////////////
// Neopixels settings //////////////////////
////////////////////////////////////////////
      
      
      int ledPin = 6; // Pin for rgb neopixel strip
      int numPixel = 15; // How many NeoPixels are attached
      int lightMenu = 1; // Default light menu item
      
      Adafruit_NeoPixel pixels = Adafruit_NeoPixel(numPixel, ledPin, NEO_GRB + NEO_KHZ800);


////////////////////////////////////////////
// Battery Voltage Divider /////////////////
////////////////////////////////////////////

      
      //#define BATTERY_METER // Uncomment if using battery voltage divider
      
      int batteryPin = A3; // Sensor value 0-1023
      
      float R1 = 472000.0; // R1 resistor (470K)
      float R2 = 111000.0; // R2 resistor (10K + 100K)
      float divider_output = 4.04; // Set this to the measured max output voltage from divider

      // Storage variables // 
      float battery = 0.0; 
      float vOut = 0.0; // Output voltage
      float vIn = 0.0; // Calculated input voltage
      int battery_loop_counter = 0;


////////////////////////////////////////////
// Misc ////////////////////////////////////
////////////////////////////////////////////

      
      int buzzerPin = 11; // Pin for buzzer output
      LiquidCrystal_I2C lcd(0x3F,16,2);  // set the LCD address to 0x3F for 16 chars and 2 line display


////////////////////////////////////////////
// PID function ////////////////////////////
////////////////////////////////////////////
      
      
      float get_pid(float angle){
        
        float delta_t = millis() - lastTime; // Delta time
        lastTime = millis(); // Reset timer

        error = setpoint - angle; // Calculate error, e=SP-Y

        
        /*
          // Throttle throttle_expo function if near max roll
          if (angle > (setpoint + pushback_angle) && throttle_expo >= -255){
            throttle_expo = -pow(throttle_expo_factor, abs(error) - pushback_angle);
          }
          else if (angle < (setpoint - pushback_angle) && throttle_expo <= 255){
            throttle_expo = pow(throttle_expo_factor, abs(error) - pushback_angle);
          }
          else throttle_expo = 0;
        */

      
        // Calculate PID
        
        // Propotional
        p_term = error;  

        // Derivative
        d_term = kd * ((error - last_error) / delta_t); 


        // Calculate output
        output = kp * (p_term + d_term); 

      
        // Limit output
        if (output > Umax){
          output = Umax;
        }
        else if (output < Umin){
          output = Umin;
        }

        /*total_output = output + throttle_expo;
        
        // Limit output
        if (total_output > 255){
          total_output = 255;
        }
        else if (total_output < -255){
          total_output = -255;
        }*/

        // Remember error for next time
        last_error = error; 
      
        return output;
      }
      
      
////////////////////////////////////////////
// Motor control ///////////////////////////
////////////////////////////////////////////
      
      
      void motor(int pwm, float angle){ // pwm = Power, angle = Forward/backward

        if (motor_direction_forward == true){
          if (angle > setpoint){
            pwmWrite(LPWM, abs(pwm));
          }
          else if (angle < setpoint){ 
            pwmWrite(RPWM, abs(pwm));
          }
        }
        else{
          if (angle > setpoint){
            pwmWrite(RPWM, abs(pwm));
          }
          else if (angle < setpoint){ 
            pwmWrite(LPWM, abs(pwm));
          }
        }
      }


////////////////////////////////////////////
// Get Kalman filtered angle from gyro /////
////////////////////////////////////////////


      float get_angle(){ 
        
        // Update all the values
        while (i2cRead(0x3B, i2cData, 14));
        accX = ((i2cData[0] << 8) | i2cData[1]);
        accY = ((i2cData[2] << 8) | i2cData[3]);
        accZ = ((i2cData[4] << 8) | i2cData[5]);
        tempRaw = (i2cData[6] << 8) | i2cData[7];
        gyroX = (i2cData[8] << 8) | i2cData[9];
        gyroY = (i2cData[10] << 8) | i2cData[11];
        gyroZ = (i2cData[12] << 8) | i2cData[13];
      
        double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
        timer = micros();
      
        // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
        // atan2 Outputs the value of -π to π (radians) - see http://en.wiKipedia.org/wiKi/Atan2
        // It is then converted from radians to degrees
        #ifdef RESTRICT_PITCH // Eq. 25 and 26
        double roll  = atan2(accY, accZ) * RAD_TO_DEG;
        double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
        #else // Eq. 28 and 29
        double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
        double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
        #endif
      
        double gyroXrate = gyroX / 131.0; // Convert to deg/s
        double gyroYrate = gyroY / 131.0; // Convert to deg/s
      
        #ifdef RESTRICT_PITCH
        // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
        if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) 
        {
          kalmanX.setAngle(roll);
          compAngleX = roll;
          kalAngleX = roll;
          gyroXangle = roll;
        } 
        else kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
      
        if (abs(kalAngleX) > 90)
          gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
          kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
        
        #else
        // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
        if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)){
          kalmanY.setAngle(pitch);
          compAngleY = pitch;
          kalAngleY = pitch;
          gyroYangle = pitch;
        } 
        else kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter
      
        if (abs(kalAngleY) > 90)
          gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
          kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
        #endif
      
        //gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
        //gyroYangle += gyroYrate * dt;
        gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
        gyroYangle += kalmanY.getRate() * dt;
      
        compAngleX = 0.995 * (compAngleX + gyroXrate * dt) + 0.005 * roll; // Calculate the angle using a Complimentary filter
        compAngleY = 0.995 * (compAngleY + gyroYrate * dt) + 0.005 * pitch;
      
        // Reset the gyro angle when it has drifted too much
        if (gyroXangle < -180 || gyroXangle > 180)
          gyroXangle = kalAngleX;
        if (gyroYangle < -180 || gyroYangle > 180)
          gyroYangle = kalAngleY;
          
        return(compAngleX);
      }



////////////////////////////////////////////
// Medianfilter ////////////////////////////
////////////////////////////////////////////
      
      
      #define NUM_READS 10 // How many values to process
      
      float medianfilter(int sensorpin){ // Returns the median of the input sensor
        
         // Read multiple values and sort them to take the mode
         int sortedValues[NUM_READS];
         for(int i=0;i<NUM_READS;i++){
           int value = analogRead(sensorpin);
           int j;
           if(value<sortedValues[0] || i==0){
              j=0; // Insert at first position
           }
           else{
             for(j=1;j<i;j++){
                if(sortedValues[j-1]<=value && sortedValues[j]>=value){
                  // j is insert position
                  break;
                }
             }
           }
           for(int k=i;k>j;k--){
             // Move all values higher than current reading up one position
             sortedValues[k]=sortedValues[k-1];
           }
           sortedValues[j]=value; // Insert current reading
         }
         // Return scaled mode of 10 values
         float returnval = 0;
         for(int i=NUM_READS/2-5;i<(NUM_READS/2+5);i++){
           returnval +=sortedValues[i];
         }
         returnval = returnval/10;
         
         return returnval*1100/1023;
      }


////////////////////////////////////////////
// Read battery voltage ////////////////////
////////////////////////////////////////////
     
      
      float read_voltage(){
        
        // Read battery voltage input and convert to 0-24v
        int battery_Read = medianfilter(batteryPin);
      
        vOut = (battery_Read * divider_output) / 1023.0; // Output voltage divider value
        vIn = vOut / (R2 / (R1 + R2)); // Input calculation
      
        //int battery_remap = map(vIn*10, 0, 252, 0, 100);  // Remap voltage to 0-100%

        if(vIn <= 21.6){
          for(int i1=8;i1<numPixel;i1++){
            pixels.setPixelColor(i1, pixels.Color(255,0,0)); // Set color
          }
          pixels.show(); // Send updated pixel color value to hardware
        }
      
        return vIn;
      }


////////////////////////////////////////////
// Startup menu ////////////////////////////
////////////////////////////////////////////


      void startup_menu(){ 

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Lights: Green  ");
        lcd.setCursor(0, 1);
        lcd.print("Frequency: 4kHz       ");
        
        while(int(setpoint - get_angle()) != 0){
          
          get_angle(); // Get angle from gyro

          // Read time menubutton is pressed
          if(digitalRead(menu_button_pin) == HIGH){
            int button_timer1 = millis();
            
            while(digitalRead(menu_button_pin) == HIGH){
              int button_timer2 = millis();

              // If button has been pressed for more than one sec, change frequency mode
              if((button_timer2 - button_timer1) > 1000){

                // Cycle frequency menu
                if(menu_item >= 3)
                  menu_item = 0;
                else
                  menu_item++;
                
                tone(buzzerPin, 2000, 300);
                
                lcd.setCursor(0, 1);
                
                switch (menu_item){ // Change frequency mode
                  case 0:{
                    lcd.print("Frequency: 2kHz       ");
                    frequency = 2000;            
                  }
                  break;
                  case 1:{
                    lcd.print("Frequency: 1kHz       ");
                    frequency = 1000;
                  }
                  break;
                  case 2:{
                    lcd.print("Frequency: 600Hz       ");
                    frequency = 600;
                  }
                  break;
                  case 3:{
                    lcd.print("Frequency: 4kHz       ");
                    frequency = 4000;
                  }
                  break;
                }
                
                // Checks if frequency is changed sucessfully
                bool success_pwm_1 = SetPinFrequencySafe(RPWM, frequency); 
                bool success_pwm_2 = SetPinFrequencySafe(LPWM, frequency);
                while(!success_pwm_1 || !success_pwm_2);
                
                delay(500);
                break;
              }
            }
            
            int button_timer2 = millis();
            if((button_timer2 - button_timer1) > 1000);


            else{ // If short press on menu button, change light mode
              
              // Cycle light menu
              if(lightMenu >= 3)
                lightMenu = 0;
              else
                lightMenu++;
                              
              tone(buzzerPin, 2000, 100);
              
              lcd.setCursor(0, 0);
              
              switch(lightMenu){ // Change lights
                case 0:{
                  lcd.print("Lights: Off  ");
                  for(int i1=0;i1<numPixel;i1++){
                    pixels.setPixelColor(i1, pixels.Color(0,0,0)); // Set color none
                  }
                }
                break;
                case 1:{
                  lcd.print("Lights: Green");
                  for(int i1=0;i1<numPixel;i1++){
                    pixels.setPixelColor(i1, pixels.Color(0,200,0)); // Set color green
                  }
                  for(int i1=8;i1<numPixel;i1++){
                    pixels.setPixelColor(i1, pixels.Color(200,200,200)); // Set color white
                  }
                }
                break;
                case 2:{
                  lcd.print("Lights: Blue ");
                  for(int i1=0;i1<8;i1++){
                    pixels.setPixelColor(i1, pixels.Color(0,0,200)); // Set color blue
                  }
                  for(int i1=8;i1<numPixel;i1++){
                    pixels.setPixelColor(i1, pixels.Color(200,200,200)); // Set color white
                  }
                }
                break;
                case 3:{
                  lcd.print("Lights: Red  ");
                  for(int i1=0;i1<8;i1++){
                    pixels.setPixelColor(i1, pixels.Color(200,0,0)); // Set color red
                  }
                  for(int i1=8;i1<numPixel;i1++){
                    pixels.setPixelColor(i1, pixels.Color(200,200,200)); // Set color white
                  }
                }
                break;
              }
              pixels.show();  // Send updated pixel color value to hardware 
           
              delay(200);
            }                      
          }
          // Prints angle from setpoint
          lcd.setCursor(14, 0);
          lcd.print(int(get_angle() - setpoint));
          lcd.print("   ");
        }
        lcd.clear();
      }


////////////////////////////////////////////
// Main setup //////////////////////////////
////////////////////////////////////////////
      
      
      void setup(){ 
        
        // Initialize lcd display
        lcd.begin();
        lcd.clear();      
        lcd.setCursor(0, 0);
        lcd.print("  SB Unicycle  "); // Intro text
        lcd.setCursor(0, 1);
        lcd.print("   Booting...  ");
        lcd.backlight();

        
        tone(buzzerPin, 2000, 80); // Startup sound
        delay(100);
        tone(buzzerPin, 2000, 80);
        delay(100);
        tone(buzzerPin, 2000, 80);
        delay(100);
        tone(buzzerPin, 2400, 80);

        
        Serial.begin(115200);
        Wire.begin();

        delay(80);

        pixels.begin(); // initialize the NeoPixel library

        
        for(int i1=8;i1<10;i1++){
          for(int i2=0;i2<200;i2++){
            pixels.setPixelColor(i1, pixels.Color(i2,i2,i2)); // Set color blue
            pixels.show();
          }
        }
        for(int i1=0;i1<4;i1++){
          for(int i2=0;i2<200;i2++){
            pixels.setPixelColor(i1, pixels.Color(0,i2,0)); // Set color blue
            pixels.setPixelColor(7-i1, pixels.Color(0,i2,0)); // Set color blue
            pixels.setPixelColor(10+i1, pixels.Color(i2,i2,i2)); // Set color blue
            pixels.show();
          }
        }
        for(int i1=14;i1<numPixel;i1++){
          for(int i2=0;i2<200;i2++){
            pixels.setPixelColor(i1, pixels.Color(i2,i2,i2)); // Set color blue
            pixels.show();
          }
        }
        
    
        // Gyro/I2C/Kalman startup
        TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz
  
        i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
        i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
        i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
        i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
        while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
        while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
      
        while (i2cRead(0x75, i2cData, 1));
        if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
          Serial.print(F("Error reading sensor"));
          while (1);
        }
      
        delay(100); // Wait for sensor to stabilize
      
        // Set kalman and gyro starting angle
        while (i2cRead(0x3B, i2cData, 6));
        accX = (i2cData[0] << 8) | i2cData[1];
        accY = (i2cData[2] << 8) | i2cData[3];
        accZ = (i2cData[4] << 8) | i2cData[5];
      
        // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
        // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
        // It is then converted from radians to degrees
        #ifdef RESTRICT_PITCH // Eq. 25 and 26
          double roll  = atan2(accY, accZ) * RAD_TO_DEG;
          double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
        #else // Eq. 28 and 29
          double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
          double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
        #endif
      
        kalmanX.setAngle(roll); // Set starting angle
        kalmanY.setAngle(pitch);
        gyroXangle = roll;
        gyroYangle = pitch;
        compAngleX = roll;
        compAngleY = pitch;
        
        timer = micros();
  
      
        // Battery voltage
        pinMode(batteryPin, INPUT);


        // initialize all timers except for 0, to save time keeping functions
        InitTimersSafe(); 
        
        // sets the frequency for the motor pwm pins
        bool success_pwm_1 = SetPinFrequencySafe(RPWM, frequency); 
        bool success_pwm_2 = SetPinFrequencySafe(LPWM, frequency);
        while(!success_pwm_1 || !success_pwm_2);
        
      
        // Motorcontroller output
        pinMode(RPWM, OUTPUT); // PWM output right channel
        pinMode(LPWM, OUTPUT); // PWM output left channel
        pinMode(R_EN, OUTPUT); // Enable right channel
        pinMode(L_EN, OUTPUT); // Enable left channel
        digitalWrite(RPWM, LOW); // Disable motor @ start
        digitalWrite(LPWM, LOW);
        digitalWrite(R_EN, LOW);
        digitalWrite(L_EN, LOW);
        
        
        // Add some initial gyro angle values
        for (int i=0; i<100; i++){
          get_angle();
        }
        
        #ifdef BATTERY_METER
          // Read initial voltage value
          battery = read_voltage();
        #endif


        // Main menu for frequency and light modes
        startup_menu();


        // Beep when setpoint is reached
        tone(buzzerPin, 2400, 300);
      }



////////////////////////////////////////////
// Main Loop ///////////////////////////////
////////////////////////////////////////////
      
      
      void loop(){ 


        #ifdef TUNING // If tuning panel is attached, read PID input
          int potP = map(analogRead(A0), 0, 1023, 0, 100);
          kp = potP;
      
          int potD = map(analogRead(A1), 0, 1023, 0, 100);
          kd = potD;
      
          int potSP = map(analogRead(A2), 0, 1023, 75, 95);
          setpoint = potSP;
        #endif


        // Get angle and calculate PID output
        angle = get_angle();        
        pid_output = get_pid(abs(angle)); 
    
  
        // If roll angle is greater than max roll or less than min roll, stop motor
        if (angle > (setpoint + max_roll) || angle < (setpoint - min_roll)){ 
        
          digitalWrite(R_EN,LOW);
          digitalWrite(L_EN,LOW);
          motor(0, angle);
            
          lcd.setCursor(0, 0);
          lcd.print(" FALL DETECTED! ");
          lcd.setCursor(0, 1);
          lcd.print("Please restart...");
            
          while(1){
            for(int i1=0;i1<4;i1++){ // Light up neopixel ledstrip
              pixels.setPixelColor(i1, pixels.Color(200,0,0)); // Set color
              pixels.setPixelColor(7-i1, pixels.Color(200,0,0)); // Set color
            }
            pixels.show(); // Send updated pixel color value to hardware
            tone(buzzerPin, 2000, 500);
            delay(500);
            
            for(int i1=0;i1<4;i1++){
              pixels.setPixelColor(i1, pixels.Color(0,0,0)); // Set color
              pixels.setPixelColor(7-i1, pixels.Color(0,0,0)); // Set color
            }
            pixels.show();  // Send updated pixel color value to hardware
            delay(500);
            }
        }

        else{
          
          // Enable and write PID output value to motor
          digitalWrite(R_EN,HIGH); 
          digitalWrite(L_EN,HIGH);
          motor(pid_output, angle); 

              
          int motorPower = int(abs(100.0f/Umax) * pid_output); // Shows motor output in % of total
          int offset = int(setpoint - angle); // Shows error from setpoint in degrees
          float offsetFine = float(angle - setpoint); // Shows error from setpoint in degrees

          //Turn on stats when angle and motor power is zero
          if(offset == 0 && motorPower == 0){enableStats = true;} 
          
          if(abs(motorPower) > maxOutput && enableStats){maxOutput = abs(motorPower);}
          else if (!enableStats){maxOutput = 0;}
    
          if(abs(offset) > maxAngle && enableStats){maxAngle = abs(offset);}
          else if (!enableStats){maxAngle = 0;}


          // Horn button
          if(digitalRead(menu_button_pin) == HIGH){tone(buzzerPin, 2400, 50);}

          // Alert if angle is 3.5 or more
          if(abs(offset) >= 3.5){tone(buzzerPin, 1600, 500);}
          
          
          // LCD output            
          lcd.setCursor(0, 0);
          lcd.print("Power:");
          lcd.print(abs(motorPower));
          lcd.print("% ");
          lcd.setCursor(10, 0);
          lcd.print("P:");
          lcd.print(abs(kp));
          
          lcd.setCursor(0, 1);
          lcd.print("Angle:");
          lcd.print(abs(offset));
          lcd.setCursor(10, 1);
          lcd.print("D:");
          lcd.print(abs(kd));
        }
      }
