#include <PID_v1.h>
#include "GY_85.h"
#include <Wire.h>
#include<math.h>

GY_85 GY85;     //create the object

#include <Encoder.h>
int offsetW=319;
double scaleW=0.512;
double delta_t=0;
double wx=0,wy=0,wz=0,tetax=0,tetay=0,tetaz=0;
double ptime=0;
unsigned count=0;
double filtered_angle=0;
double HPF=0.5, LPF=0.5;


//Define Variables we'll be connecting to
double SetpointRight, InputRight, OutputRight, SetpointLeft,InputLeft,OutputLeft,SetpointStand;
//Specify the links and initial tuning parameters
//Define the aggressive and conservative Tuning Parameters
//constantes para roda esquerda
/*double aggKp=6, aggKi=0.3, aggKd=1;
  double consKp=3, consKi=0.1, consKd=0.5;
 */



unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
unsigned int time = 2;


//double aggKp=90, aggKi=3, aggKd= -0.001;
//double consKp=80, consKi=3, consKd= -0.006;
static const unsigned long REFRESH_INTERVAL = 100; // ms
static unsigned long lastRefreshTime = 0;


//PidRight
PID RightPID(&InputRight, &OutputRight, &SetpointRight,120,1,900,P_ON_E, DIRECT);
PID LeftPID(&InputLeft, &OutputLeft, &SetpointLeft,120,1,900,P_ON_E, DIRECT);

int MOTOR_R_EN = 12;
int MOTOR_R_1 = 28;
int MOTOR_R_2 = 30;

int MOTOR_L_EN = 13;
int MOTOR_L_1 = 34;
int MOTOR_L_2 = 32;

int ENC_R_1 = 19;
int ENC_R_2 = 18;
Encoder EncR(ENC_R_1, ENC_R_2);
int ENC_L_1 = 2;
int ENC_L_2 = 3;
Encoder EncL(ENC_L_1, ENC_L_2);

long oldPositionR  = -999;
long oldPositionL  = -999;

void setup() {
	Serial.begin(115200);

	pinMode(MOTOR_R_1, OUTPUT);
	pinMode(MOTOR_R_2, OUTPUT);
	pinMode(MOTOR_L_1, OUTPUT);
	pinMode(MOTOR_L_2, OUTPUT);

	InputRight = 0;
	SetpointRight = 0;
	InputLeft = 0;
	SetpointLeft = 0;
	OutputRight=0;
	OutputLeft=0;
	SetpointStand=8;

	//turn the PID on
	RightPID.SetMode(AUTOMATIC);
	RightPID.SetSampleTime(2);
	RightPID.SetOutputLimits(-255,255);
	LeftPID.SetMode(AUTOMATIC);
	LeftPID.SetSampleTime(2);
	LeftPID.SetOutputLimits(-255,255);

	Wire.begin();
	delay(10);

	GY85.init();
	delay(10);
}

void loop() {
	currentMillis = millis();

	//analogWrite(MOTOR_R_EN, 150);
	//analogWrite(MOTOR_L_EN, 150);
	//digitalWrite(MOTOR_R_1, LOW);
	//digitalWrite(MOTOR_R_2, HIGH);

	long newPositionL = EncL.read();
	if (newPositionL != oldPositionL) {
		oldPositionL = newPositionL;
	}

	long newPositionR = EncR.read();
	if (newPositionR != oldPositionR) {
		oldPositionR = newPositionR;
	}

	if(OutputRight > 0){
		digitalWrite(MOTOR_R_1, HIGH);
		digitalWrite(MOTOR_R_2, LOW);
	}else{
		digitalWrite(MOTOR_R_1, LOW);
		digitalWrite(MOTOR_R_2, HIGH);
	}
	if(OutputLeft > 0){
		digitalWrite(MOTOR_L_1, HIGH);
		digitalWrite(MOTOR_L_2, LOW);
	}else{
		digitalWrite(MOTOR_L_1, LOW);
		digitalWrite(MOTOR_L_2, HIGH);
	}

	float gx = GY85.gyro_x( GY85.readGyro() );
	float gy = GY85.gyro_y( GY85.readGyro() );
	float gz = GY85.gyro_z( GY85.readGyro() );
	float gt = GY85.temp  ( GY85.readGyro() );
	float ax = GY85.accelerometer_x( GY85.readFromAccelerometer() )*0.015625/2;
	float ay = GY85.accelerometer_y( GY85.readFromAccelerometer() )*0.015625/2;
	float az = GY85.accelerometer_z( GY85.readFromAccelerometer() )*0.015625/2;

	tetax += gx*delta_t/1000;
	tetay += gy*delta_t/1000;
	tetaz += gz*delta_t/1000;
	float angle_accel= (acos(ax)*180/M_PI)-90;
	if (currentMillis - previousMillis >= time) {
		delta_t = currentMillis - previousMillis;
		previousMillis = currentMillis;
		double aux = HPF*( filtered_angle + (gx/14.375) * (delta_t/1000)) + LPF*(angle_accel);
		if(aux == aux) // checking for nan
			filtered_angle = aux;
		if(count>10){
			Serial.print(filtered_angle);
			Serial.print(" ");
			Serial.println(SetpointStand);
			count = 0;
		}else{
			count +=1;
		}
		if(fabs(SetpointStand-filtered_angle ) > 0){
			SetpointLeft=1.0*((SetpointStand-filtered_angle)/3.2);//+ 0.2*(newPositionR-newPositionL);
			SetpointRight=1.0*((SetpointStand-filtered_angle)/3.2);//-0.2*(newPositionR-newPositionL);
			RightPID.Compute();
			LeftPID.Compute();

			analogWrite(MOTOR_R_EN, fabs(OutputRight));
			analogWrite(MOTOR_L_EN, fabs(OutputLeft));
		}else{
			analogWrite(MOTOR_R_EN, 0);
			analogWrite(MOTOR_L_EN, 0);
			Serial.println("MotorOFF");

		}
	}

}
