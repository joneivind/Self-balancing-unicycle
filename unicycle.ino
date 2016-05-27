#define pi = 3.14159265359; //Value for pi
#define dt = (10.0/1000.0) //100hz = 10ms

//PID constants
float kp = 5.0;
float ki = 2.0;
float kd = 2.0;

float setpoint = 0; //Initial setpoint
int deadband = 2; //+-degrees of deadband around setpoint
int max_pitch = 15; //Degrees before motor cut *SAFETY FUNCTION*
int max_roll = 15; //Degrees before motor cut *SAFETY FUNCTION*

//PID variables
int Umax = 255;	 //Max output
int Umin = -255; //Min output

float p_term = 0; //Store propotional value
float i_term = 0; //Store integral value
float d_term = 0; //Store derivative value

float error = 0; //Sum error
float last_error = 0; //Store last error sum

int output = 0;	//PID output

//Timers
int timer = millis();
int lastTime = millis();

//Input output pins
int gyroPin = 4;
int TN1 = 4;
int TN2 = 3;
int ENA = 5;
int TN3 = 8;
int TN4 = 7;
int ENB = 6;


//PID function -> Calculate output from angle input
float pid(float measured_angle)
{
	float delta_t = (millis() - lastTime); //Delta time
	lastTime = millis(); //Reset timer

	error = setpoint - measured_angle; //Calculate error

	p_term = kp * error;	//Propotional
	i_term += (ki * error * delta_t);	//Integral
	d_term = kd * ((error - last_error) / delta_t); //Derivative

	output = p_term + i_term + d_term; //Calculate output

	//Limit output
	if (output > Umax) output = Umax;
	else if (output < Umin) output = Umin;
	
	last_error = error; //Remember error for next time

	return output;
}


//Motor output
void motor(int pwm, float angle_pitch)
{
	//Set direction
	if (angle_pitch > (setpoint + deadband))
	{
		//Forwards
	}
	else if (angle_pitch < (setpoint - deadband))
	{
		//Backwards
	}
	else
	{
		//Stop motor
	}

	analogWrite(enA, abs(pwm)); //Writes output to motor
}



void setup 
{
	Serial.begin(115200);

	pinMode(gyroPin, INPUT);
	pinMode(TN1, OUTPUT);
 	pinMode(TN2, OUTPUT);
 	pinMode(TN3, OUTPUT);
 	pinMode(TN4, OUTPUT);
 	pinMode(ENA, OUTPUT);
	pinMode(ENB, OUTPUT);

	timer = millis(); //Initialize timer
}



void loop 
{
	if ((millis() - timer) > (dt * 1000)) //100hz
	{
	       	timer = millis(); //Reset timer
	
	       	//Read angels from MCU
	    	float filtered_angle_pitch = analogRead(gyroPin);
	    	float filtered_angle_roll = analogRead(gyroPin);
	
	    	//If angle xy is greater than max degrees, stop motors
	    	if ((abs(filtered_angle_pitch) < max_pitch) && (abs(filtered_angle_roll) < max_roll))
	    	{
			int u_output = pid(filtered_angle_pitch);
			motor(u_output, filtered_angle_pitch);
		}
		else
	    	{	
			motor(0, filtered_angle_pitch); //Stop motor
		}
	}
}
