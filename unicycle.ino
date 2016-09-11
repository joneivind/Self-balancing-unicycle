/*

  ***************************************
  ******* Self balancing unicycle *******
  ***************************************

  Final year project in electrical engineering @ Høyskolen i Oslo Akershus, Norway.
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

  * Ledstrip
  * VCC     +5v
  * GND     GND
  * SIGNAL  D6

  * Buzzer
  * SIGNAL(+) D11
  * GND     GND
  
  * Reset button
  * SIGNAL  D4          D4
  * SIGNAL_HIGH D12
  * LED     D5          D5
  * GND     GND         GND
  
  * Voltage divider 0-24v -> 0-5v ( R1: 470k, R2: 100k + 10k)
  * Vout    A3          A3    Vout + from battery

  ***** Connections end *****

  Credits 
  MPU6050 code: http://www.pitt.edu/~mpd41/Angle.ino
  Softwarefilter: http://www.elcojacobs.com/eleminating-noise-from-sensor-readings-on-arduino-with-digital-filtering/

*/


////////////////////////////////////////////
// Imports /////////////////////////////////
////////////////////////////////////////////


      #include <Wire.h>
      #include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
      #include <LiquidCrystal_I2C.h>
      #include <Adafruit_NeoPixel.h>
      #include <avr/power.h>
      
      LiquidCrystal_I2C lcd(0x3F,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display


////////////////////////////////////////////
// PID constants ///////////////////////////
////////////////////////////////////////////


      float kp = 50.0;
      float ki = 0.0; 
      float kd = 7.0;

      //#define TUNING // Uncomment if tuning panel is attached


////////////////////////////////////////////
// PID output variable /////////////////////
////////////////////////////////////////////
      
      float setpoint = 81.0; // Initial degree setpoint
      
      bool ki_enable = false; //Enables integral regulator if true, disabled if false
      
      float deadband = 0.0; // +-degrees of deadband around setpoint where motor output is zero
      int max_roll = 15; // Degrees from setpoint before motor will stop
      int min_roll = 8; // Degrees from setpoint before motor will stop


////////////////////////////////////////////
// Pushback function ///////////////////////
////////////////////////////////////////////


      float pushback_angle = 1.8;  // Degrees where pushback should activate *Must be less than max_roll*
      float pushback_range = 1.8;  // Degrees from setpoint where pushback deactivates if activated
      int expo = 0;
      
      bool motor_direction_forward = false;  // Set motor direction forward/reverse
      
      bool fall_detection_trigger = false; // Default value fall detection *DONT CHANGE*


////////////////////////////////////////////
// PID variables ///////////////////////////
////////////////////////////////////////////


      float Umax = 255.0;  // Max output
      float Umin = -255.0; // Min output
      
      float p_term = 0.0; // Store propotional value
      float i_term = 0.0; // Store integral value
      float d_term = 0.0; // Store derivative value
      
      float error = 0.0; // Sum error
      float last_error = 0.0; // Store last error sum
      
      int output = 0; // PID output + expo
      int pid_output = 0;


////////////////////////////////////////////
// Stats ///////////////////////////////////
////////////////////////////////////////////


      bool enableStats = false;
      int maxOutput = 0;
      int maxAngle = 0;


////////////////////////////////////////////
// Timers //////////////////////////////////
////////////////////////////////////////////


      int main_loop_timer = millis(); // Dt timer main loop 
      int lastTime = millis(); //Dt timer PID loop


////////////////////////////////////////////
// Input voltage divider ///////////////////
////////////////////////////////////////////


      int battery_voltage_input = A3; // Sensor value 0-1023


////////////////////////////////////////////
// Motor Driver BTS7960 pins ///////////////
////////////////////////////////////////////


      int RPWM = 9; // Forward pwm input
      int LPWM = 10; // Reverse pwm input
      int R_EN = 7; // Forward drive enable input
      int L_EN = 8; // Reverse drive enable input


////////////////////////////////////////////
// Reset button ////////////////////////////
////////////////////////////////////////////


      int reset_button_pin = 4;
      int reset_button_led_pin = 5;
      int reset_button_constant_high = 12;
      bool reset_button_led_state = true;
      int led_counter = 0;


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
      
      
      int ledPin = 6;
      int numPixel = 15; // How many NeoPixels are attached to the Arduino?
      
      Adafruit_NeoPixel pixels = Adafruit_NeoPixel(numPixel, ledPin, NEO_GRB + NEO_KHZ800);


////////////////////////////////////////////
// Misc ////////////////////////////////////
////////////////////////////////////////////
      
      int buzzerPin = 11;
      
      float battery = 0.0;
      int batteryCounter = 0;
      int BattyLedOn = 1;


////////////////////////////////////////////
// PID function ////////////////////////////
////////////////////////////////////////////
      
      
      float get_pid(float angle)
      {
        float delta_t = millis() - lastTime; // Delta time
        lastTime = millis(); // Reset timer
      
        error = setpoint - angle; // Calculate error, e=SP-Y

      
        // Pushback function if near max roll
        if (angle > (setpoint + pushback_angle) && expo >= -255) 
        {
          expo = -pow(10, abs(error) - pushback_angle);
        }
        else if (angle < (setpoint - pushback_angle) && expo <= 255) 
        {
          expo = pow(10, abs(error) - pushback_angle);
        }
        else
        {
          expo = 0;
        }

      
       // Calculate PID
        p_term = error;  // Propotional
        
        if (ki_enable == true) 
        {  
          i_term += (ki * error * delta_t); // Integral
        }
        else i_term = 0; // Disable integral regulator

        
        d_term = kd * ((error - last_error) / delta_t); // Derivative

      
        output = kp * (p_term + i_term + d_term) + expo; // Calculate output

      
        // Limit output
        if (output > 255) 
        {
          output = 255;
        }
        else if (output < -255) 
        {
          output = -255;
        }


        /*total_output = output + expo;


        // Limit output
        if (total_output > 255) 
        {
          total_output = 255;
        }
        else if (total_output < -255) 
        {
          total_output = -255;
        }*/

        
        last_error = error; // Remember error for next time

      
        return output;
      }
      
      
////////////////////////////////////////////
// Motor control ///////////////////////////
////////////////////////////////////////////
      
      
      void motor(int pwm, float angle)
      {
        if (motor_direction_forward == true)
        {
          if (angle > (setpoint + deadband))
          { 
            analogWrite(LPWM, abs(pwm)); // Drive backward
          }
          else if (angle < (setpoint - deadband))
          { 
            analogWrite(RPWM, abs(pwm)); // Drive forward
          }
        }
        else{
          if (angle > (setpoint + deadband))
          { 
            analogWrite(RPWM, abs(pwm)); // Drive backward
          }
          else if (angle < (setpoint - deadband))
          { 
            analogWrite(LPWM, abs(pwm)); // Drive forward
          }
        }
      }


////////////////////////////////////////////
// Get Kalman filtered angle from gyro /////
////////////////////////////////////////////


      float get_angle() 
      { 
        /* Update all the values */
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
        if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) 
        {
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
      
        compAngleX = 0.99 * (compAngleX + gyroXrate * dt) + 0.01 * roll; // Calculate the angle using a Complimentary filter
        compAngleY = 0.99 * (compAngleY + gyroYrate * dt) + 0.01 * pitch;
      
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
      
      
      #define NUM_READS 100
      float medianfilter(int sensorpin){
         // read multiple values and sort them to take the mode
         int sortedValues[NUM_READS];
         for(int i=0;i<NUM_READS;i++){
           int value = analogRead(sensorpin);
           int j;
           if(value<sortedValues[0] || i==0){
              j=0; //insert at first position
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
             // move all values higher than current reading up one position
             sortedValues[k]=sortedValues[k-1];
           }
           sortedValues[j]=value; //insert current reading
         }
         //return scaled mode of 10 values
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
     
      
      float read_voltage()
      {
        // Read battery voltage input and convert to 0-24v
        int battery_Read = medianfilter(battery_voltage_input);
      
        float battery_voltage = (5.0 / 1023.0) * battery_Read * 4.5 * 10.0; // Calculate real voltage value * 10
      
        int battery_remap = map(battery_voltage, 0, 252, 0, 100);  // Remap voltage to 0-100%
        int battery_remap_led = map(battery_voltage, 204, 234, 0, 4);  // Remap voltage to 0-100%
        
        if(battery_voltage <= 21.6)
        {
          for(int i1=8;i1<numPixel;i1++)
          {
            pixels.setPixelColor(i1, pixels.Color(255,0,0)); // Set color
          }
          pixels.show();  // Send updated pixel color value to hardware
        }
      
        return battery_voltage;
      }


////////////////////////////////////////////
// Main setup //////////////////////////////
////////////////////////////////////////////
      
      
      void setup()
      { 
        Serial.begin(115200);
        Wire.begin();
            
        // Initialize lcd display
        lcd.begin();
        lcd.clear(); // Clear display
        lcd.setCursor(0, 0);
        lcd.print("  SB Unicycle  ");
        lcd.setCursor(0, 1);
        lcd.print("  Booting...   ");
        lcd.backlight(); // Turn on backlight
      
      
        pixels.begin(); // initialize the NeoPixel library

    
        ////////// Gyro / I2C / Kalman startup ///////////
        
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
      
        /* Set kalman and gyro starting angle */
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
        pinMode(battery_voltage_input, INPUT);


        // Change PWM frequency (Affects servo timer)
        TCCR1B = TCCR1B & B11111000 | B00000010; // set timer 1 divisor to 8 for PWM frequency of 3921.16 Hz
      
      
        // Motorcontroller
        pinMode(RPWM, OUTPUT); // PWM output right channel
        pinMode(LPWM, OUTPUT); // PWM output left channel
        pinMode(R_EN, OUTPUT); // Enable right channel
        pinMode(L_EN, OUTPUT); // Enable left channel
        digitalWrite(RPWM, LOW); // Disable motor @ start
        digitalWrite(LPWM, LOW);
        digitalWrite(R_EN, LOW);
        digitalWrite(L_EN, LOW);
        
        
        // Reset button
        pinMode(reset_button_pin, INPUT);
        pinMode(reset_button_led_pin, OUTPUT);
        pinMode(reset_button_constant_high, OUTPUT);
        digitalWrite(reset_button_constant_high, HIGH);
      
        
        // Initialize timer
        main_loop_timer = millis(); // Initialize main loop timer
        
        
        // Add some initial gyro angle values
        for (int i=0; i<100; i++)
        {
          get_angle();
        }
      
        
        // Light up neopixel ledstrip
        
        for(int i1=0;i1<4;i1++)
        {
          pixels.setPixelColor(i1, pixels.Color(0,55,0)); // Set color
          pixels.setPixelColor(7-i1, pixels.Color(0,55,0)); // Set color
          pixels.setPixelColor(i1+8, pixels.Color(100,100,100)); // Set color
          pixels.show();  // Send updated pixel color value to hardware
        }
        for(int i1=12;i1<numPixel;i1++)
        {
          pixels.setPixelColor(i1, pixels.Color(100,100,100)); // Set color
          pixels.show();  // Send updated pixel color value to hardware
        }
      
        //tone(buzzerPin, 1000, 500);
      
        // Read initial voltage value
        battery = read_voltage();
      
      
        /*
        // Turn on light in reset button
        for(int i=0;i<255;i++)
        {
          analogWrite(reset_button_led_pin, i);
          delay(2);
        }
        */
          
        
        // Waiting for upright position
        lcd.clear();
        while(int(setpoint - get_angle()) != 0)
        {
          get_angle();
          lcd.setCursor(0, 0);
          lcd.print("Waiting for pos.");
          lcd.setCursor(0, 1);
          lcd.print("Offset: ");
          lcd.print(int(get_angle() - setpoint));
          lcd.print("   ");
        }
        lcd.clear();
      
        //tone(buzzerPin, 1300, 1000);  
      }



////////////////////////////////////////////
// Main Loop ///////////////////////////////
////////////////////////////////////////////
      
      
      void loop()
      { 
        if ((millis() - main_loop_timer) > (2.0f/1000.0f * 1000)) // Run loop @ 100hz (1/100hz = 10ms) - 1/400hz = 2.5ms - 1/500hz = 2ms
        {
          main_loop_timer = millis(); // Reset main loop timer

      
          #ifdef TUNING
            int potP = map(analogRead(A0), 0, 1023, 0, 100);
            kp = potP;
        
            int potD = map(analogRead(A1), 0, 1023, 0, 100);
            kd = potD;
        
            //int potSP = map(analogRead(A2), 0, 1023, 75, 95);
            //setpoint = potSP;
          #endif

      
          float angle = get_angle();


          pid_output = get_pid(abs(angle)); // Calculate PID output

      
          if (angle > (setpoint + max_roll) || angle < (setpoint - min_roll)) // If roll angle is greater than max roll, stop motor
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
                
              while(1)
              { 
                
                // Fade down leds
                for(int i1=255;i1>0;i1--)
                {
                  for(int i2=0;i2>8;i2++)
                  {
                    pixels.setPixelColor(i2, pixels.Color(i1,0,0)); // Set color
                    pixels.show();  // Send updated pixel color value to hardware
                  }
                }
                // Fade up leds
                for(int i1=0;i1<255;i1++)
                {
                  for(int i2=0;i2>8;i2++)
                  {
                    pixels.setPixelColor(i2, pixels.Color(i1,0,0)); // Set color
                    pixels.show();  // Send updated pixel color value to hardware
                  }
                }
              }
            }
          }
          
          else
          { 
            digitalWrite(R_EN,HIGH);
            digitalWrite(L_EN,HIGH);
            motor(pid_output, angle); // Enable and write PID output value to motor
      
            
            int motorPower = int(abs(100.0f/Umax) * pid_output); // Shows motor output in % of total
            int offset = int(setpoint - angle); // Shows error from setpoint in degrees
            float offsetFine = float(angle - setpoint); // Shows error from setpoint in degrees
      
      
            //Turn on stats
            if(offset == 0 && motorPower == 0)
            {
              enableStats = true;
            }
            
            if(abs(motorPower) > maxOutput && enableStats)
            {
              maxOutput = abs(motorPower);
            }
            else if (!enableStats)
            {
              maxOutput = 0;
            }
      
            if(abs(offset) > maxAngle && enableStats)
            {
              maxAngle = abs(offset);
            }
            else if (!enableStats)
            {
              maxAngle = 0;
            }
      
            
            batteryCounter++;
            
            if(batteryCounter > 100)  // Update batterystatus every second
            {
              battery = read_voltage();
              batteryCounter = 0;
            }
      
            
            // LCD output
              
            // Offset monitor
            lcd.setCursor(9, 0);
            lcd.print("O:");
            lcd.print(offsetFine);
            lcd.print(" ");
            
            // Battery monitor
            lcd.setCursor(0, 0);
            lcd.print("B:");
            lcd.print(battery/10.0f);
            lcd.print("v");
            
            // P value
            lcd.setCursor(0, 1);
            lcd.print("P:");
            lcd.print(kp);
            lcd.print("  ");
      
            //D value
            lcd.setCursor(9, 1);
            lcd.print("D:");
            lcd.print(kd);
      
           
            // DEBUG Serial display
            /*Serial.print(setpoint);
            Serial.print("\t");
            Serial.println(error + setpoint);*/

             Serial.print(pid_output);
          Serial.print("\t");
          Serial.println(offsetFine);
             
            //Serial.print("\t");
            //Serial.println(angle);
                
            // ***** LCD output end *****
      
          }
        }
      }
