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
  MPU6050/Complimentary filter: http://www.pitt.edu/~mpd41/Angle.ino
  Medianfilter: http://www.elcojacobs.com/eleminating-noise-from-sensor-readings-on-arduino-with-digital-filtering/

*/


////////////////////////////////////////////
// Library /////////////////////////////////
////////////////////////////////////////////


      #include <Wire.h>
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
      float main_loop_timer = millis();


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

     
      const int MPU_addr=0x68;
      double AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //These will be the raw data from the MPU6050.
      uint32_t timer; //it's a timer, saved as a big-ass unsigned int.  We use it to save times from the "micros()" command and subtract the present time in microseconds from the time stored in timer to calculate the time for each loop.
      double compAngleX, compAngleY; //These are the angles in the complementary filter
      #define degconvert 57.2957786 //there are like 57 degrees in a radian.


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
        
        //Collect raw data from the sensor.
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
        compAngleX = 0.993 * (compAngleX + gyroXrate * dt) + 0.007 * roll; // Calculate the angle using a Complimentary filter
        //compAngleY = 0.993 * (compAngleY + gyroYrate * dt) + 0.007 * pitch; 
          
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

        int startangle = 0;

        for(int i=0;i<700;i++){
          startangle = get_angle();
          lcd.setCursor(14, 0);
          lcd.print(startangle);
          lcd.print("   ");
          delay(1);
        }
        
        while(startangle != setpoint){
          
          startangle = get_angle(); // Get angle from gyro

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
          lcd.print(startangle);
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
        
    
        // Set up MPU 6050:
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
      
        //start a timer
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
        
        
        /*// Add some initial gyro angle values
        for (int i=0; i<1000; i++){
          get_angle();
        }*/
        
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

      if((millis() - main_loop_timer) >= 0.0){ // Looptime as fast as possible
        
        int looptime = millis() - main_loop_timer;
        main_loop_timer = millis(); // Reset main loop timer

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

        Serial.println(angle);
        //Serial.print("\t");
        //Serial.println(looptime);
    
        
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
      }
