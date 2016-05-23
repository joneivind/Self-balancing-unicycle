#include <Kalman.h>

// Rutine for repeat part of the code every X miliseconds
#define runEvery(t) for (static long _lasttime;\
                         (uint16_t)((uint16_t)millis() - _lasttime) >= (t);\
                         _lasttime += (t))

//Set up a timer Variable
uint32_t timer;

double InputRoll;    // Roll angle value
double InitialRoll;  // Roll initial angle

void setup() {
  
  Serial.begin(9600);
  
  timer = micros();       // Initialize timer
  
  
  InitSensors();          // Initialize sensors
  InitMotors();           // Initialize motors
  delay(1000);            // Wait until sensors be ready
  
  InitialValues();        // Get the initial angle values
  
  SetSetpoint(0);         // Set the PID setpoint to 0

}

double RollAngle =0; // Roll angle variable

void loop() {
  
  runEvery(10){         // Exetutes this part of the code every 10 miliseconds -> 100Hz 
       
    RollAngle = 0.98* (RollAngle + getDGyroRoll()) + 0.02 * (getAccelRoll());
    timer = micros();    // Reset the timer
    
    // Uncomment this for print data to Graphics
    /*
    Serial.write((byte)57);                // security value
    Serial.write((byte)getAccelRoll());    // Accel Roll value
    Serial.write((byte)getGyroRoll);    // Gyro Roll value
    Serial.write((byte)RollAngle-InitialRoll);    // Real Roll
    */
    
    MotorControl(Compute(RollAngle-InitialRoll));  // Sends the real roll angle -> Roll - InitialRoll
      
  }
  
  ReceiveData();      // Checks Serial for incoming data
  
}

// This function is used to debug the robot, changig the set 
// value of the PID
void ReceiveData(){
  
  if (Serial.available()){
    char a = Serial.read();
    switch (a){
      case 'q':
        SetSetpoint(GetSetPoint()+0.01);
      break;
      case 'w':
        SetSetpoint(GetSetPoint()-0.01);
      break;
      case 'e':
        SetSetpoint(GetSetPoint()+0.1);
      break;
      case 'r':
        SetSetpoint(GetSetPoint()-0.1);
      break;
      case 's':
        InitialRoll = RollAngle;
        Show(RollAngle);
      break;
    } 
    
  }
  
}

// Show data via serial port
void Show(double a){
 
  Serial.print("DATOS: ");
  Serial.print("Initial Roll: ");
  Serial.print(InitialRoll);
  Serial.print("Roll: ");
  Serial.print(a);
  Serial.print("Real Roll: ");
  Serial.println(RollAngle-InitialRoll);
  
}







