// AIR ROVER smart car
// goldelec.com
// This code is public domain, enjoy!


#include <avr/io.h>
#include "config.h"
#include "def.h"
#include <avr/pgmspace.h>
#include <SoftwareSerial.h>
#include <math.h>

#include <Servo.h>
#include "AFMotor.h"



#if DEUBUG_ON

SoftwareSerial exSerial(2, 13); // RX, TX


#define logPrint(data, args...)   exSerial.print(data,   ## args)
#define logPrintln(data, args...) exSerial.println(data, ## args)
#else
#define logPrint(data, args...)   
#define logPrintln(data, args...) 
#endif


uint16_t failsafeCnt = 3;

enum rc {
    ROLL,
    PITCH,
    YAW,
    THROTTLE,
    AUX1,
    AUX2,
    AUX3,
    AUX4
};


//ROLL, PITCH, YAW, AUX1, AUX2, AUX3, AUX4
volatile uint16_t serialRcValue[RC_CHANS] = {1502, 1502, 1502, 1000, 1502, 1502, 1502, 1502};

static int16_t rcData[RC_CHANS];    // interval [1000;2000]     ROLL, PITCH, YAW, THROTTLE, AUX1, AUX2, AUX3, AUX4



typedef enum {
  DEVICE_STATE_IDLE = 0,
  DEVICE_STATE_DONE = 5, 
} DEVICE_STATE;

DEVICE_STATE deviceState;

#define STATUS_LED 13
#define STATUS_LED_BLINK_DURATION  500
#define STATUS_DEVICE_STICK_DURATION  50
#define STATUS_HB_DURATION  1000


unsigned long deviceTime = 0;
unsigned long hbTime = 0;



static uint32_t currentTime = 0;
static uint16_t previousTime = 0;
static uint16_t cycleTime = 0;     // this is the number in micro second to achieve a full loop, it can differ a little and is taken into account in the PID loop
static uint8_t  vbat;                   // battery voltage in 0.1V steps
static int16_t  debug[4];
static int16_t  i2c_errors_count = 0;


volatile char item;
volatile int updown;
volatile int leftright;
volatile int range;
volatile int motor1Speed;
volatile int motor2Speed;
volatile int motor3Speed;
volatile int motor4Speed;
Servo servo_10;

/*
m4  m3
m1  m2
*/
AF_DCMotor motor1(1);   
AF_DCMotor motor2(2); 
AF_DCMotor motor3(3); 
AF_DCMotor motor4(4); 

void updateSpeed(int16_t ctlPower){
	int16_t targetCtlPower = ctlPower;
	if(targetCtlPower < CTL_THRESHOLD)
		targetCtlPower = CTL_THRESHOLD;

	if(targetCtlPower > 500)
		targetCtlPower = 500;

 	motor1Speed = (int)(targetCtlPower / 500.0 * MOTOR_MAX_SPEED);
  motor2Speed = (int)(targetCtlPower / 500.0 * MOTOR_MAX_SPEED);
  motor3Speed = (int)(targetCtlPower / 500.0 * MOTOR_MAX_SPEED);
  motor4Speed = (int)(targetCtlPower / 500.0 * MOTOR_MAX_SPEED);
}

void goLeft() {
/*
	motor1.setSpeed(motor1Speed);
	motor1.run(BACKWARD);
	motor2.setSpeed(motor2Speed);
	motor2.run(FORWARD);
	motor3.setSpeed(motor3Speed);
	motor3.run(FORWARD);
	motor4.setSpeed(motor4Speed);
	motor4.run(BACKWARD);
*/
	motor1.setSpeed(motor1Speed);
	motor1.run(FORWARD);
	motor2.setSpeed(motor2Speed);
	motor2.run(BACKWARD);
	motor3.setSpeed(motor3Speed);
	motor3.run(FORWARD);
	motor4.setSpeed(motor4Speed);
	motor4.run(BACKWARD);
}

void goFront() {
	motor1.setSpeed(motor1Speed);
	motor1.run(FORWARD);
	motor2.setSpeed(motor2Speed);
	motor2.run(FORWARD);
	motor3.setSpeed(motor3Speed);
	motor3.run(FORWARD);
	motor4.setSpeed(motor4Speed);
	motor4.run(FORWARD);
}

void goBack() {
	motor1.setSpeed(motor1Speed);
	motor1.run(BACKWARD);
	motor2.setSpeed(motor2Speed);
	motor2.run(BACKWARD);
	motor3.setSpeed(motor3Speed);
	motor3.run(BACKWARD);
	motor4.setSpeed(motor4Speed);
	motor4.run(BACKWARD);
}

void goRight() {
	motor1.setSpeed(motor1Speed);
	motor1.run(BACKWARD);
	motor2.setSpeed(motor2Speed);
	motor2.run(FORWARD);
	motor3.setSpeed(motor3Speed);
	motor3.run(BACKWARD);
	motor4.setSpeed(motor4Speed);
	motor4.run(FORWARD);
}

void stopMoving() {
	motor1.setSpeed(0);
	motor1.run(RELEASE);
	motor2.setSpeed(0);
	motor2.run(RELEASE);
	motor3.setSpeed(0);
	motor3.run(RELEASE);
	motor4.setSpeed(0);
	motor4.run(RELEASE);
}

void goFrontRight() {
	motor1.setSpeed(0);
	motor1.run(FORWARD);
	motor2.setSpeed(motor2Speed);
	motor2.run(FORWARD);
	motor3.setSpeed(0);
	motor3.run(FORWARD);
	motor4.setSpeed(motor3Speed);
	motor4.run(FORWARD);
}


void goFrontLeft() {
	motor1.setSpeed(motor1Speed);
	motor1.run(FORWARD);
	motor2.setSpeed(0);
	motor2.run(FORWARD);
	motor3.setSpeed(motor4Speed);
	motor3.run(FORWARD);
	motor4.setSpeed(0);
	motor4.run(FORWARD);
}


void goBackRight() {
	motor1.setSpeed(motor1Speed);
	motor1.run(BACKWARD);
	motor2.setSpeed(0);
	motor2.run(BACKWARD);
	motor3.setSpeed(motor4Speed);
	motor3.run(BACKWARD);
	motor4.setSpeed(0);
	motor4.run(BACKWARD);
}


void goBackLeft() {
	motor1.setSpeed(0);
	motor1.run(BACKWARD);
	motor2.setSpeed(motor2Speed);
	motor2.run(BACKWARD);
	motor3.setSpeed(0);
	motor3.run(BACKWARD);
	motor4.setSpeed(motor3Speed);
	motor4.run(BACKWARD);
}


void rotateLeft(){
	motor1.setSpeed(motor1Speed);
	motor1.run(BACKWARD);
	motor2.setSpeed(motor2Speed);
	motor2.run(FORWARD);
	motor3.setSpeed(motor3Speed);
	motor3.run(FORWARD);
	motor4.setSpeed(motor4Speed);
	motor4.run(BACKWARD);
}

void rotateRight(){
	motor1.setSpeed(motor1Speed);
	motor1.run(FORWARD);
	motor2.setSpeed(motor2Speed);
	motor2.run(BACKWARD);
	motor3.setSpeed(motor3Speed);
	motor3.run(BACKWARD);
	motor4.setSpeed(motor4Speed);
	motor4.run(FORWARD);
}


void setThrottle() {
  int16_t throttle = rcData[THROTTLE];
  int16_t roll     = rcData[ROLL];
  int16_t pitch    = rcData[PITCH];
  int16_t yaw      = rcData[YAW];
}

double getAngle(int x, int y) {
	double a = atan2f(y, x);
	double ret = a * 180 / M_PI; //弧度转角度，方便调试
	if (ret > 360) {
		ret -= 360;
	}
	if (ret < 0) {
		ret += 360;
	}
	
	return ret;
}

void writeMotors() {
  int16_t throttle = rcData[THROTTLE];
  int16_t roll     = rcData[ROLL];
  int16_t pitch    = rcData[PITCH];
  int16_t yaw      = rcData[YAW];

	int16_t rollDiff  = roll - 1500;
	int16_t pitchDiff = pitch - 1500;

	double ctlAngle = 0;
	int16_t ctlPower = 0;


	if((rollDiff == 0) && (pitchDiff ==0)){
		ctlAngle = 0;
		ctlPower = 0;
	}
	else{
		ctlAngle = getAngle(rollDiff, pitchDiff);
		ctlPower = (int16_t)sqrt(pow(rollDiff, 2) + pow(pitchDiff, 2));
	}

	int16_t ctlPowerFromYaw = abs(yaw - 1500);


	//logPrint("P");
	//logPrintln(ctlPower);

	if(ctlPowerFromYaw < 150){
		updateSpeed(ctlPower);
		
		if(ctlPower < 80){
			stopMoving();
		}
		else{
			if((ctlAngle >= 337.5) || (ctlAngle < 22.5)){
				goRight();
			}
			else if((ctlAngle >= 22.5) && (ctlAngle < 67.5)){
				goFrontRight();
			}
			else if((ctlAngle >= 67.5) && (ctlAngle < 112.5)){
				goFront();
			}
			else if((ctlAngle >= 112.5) && (ctlAngle < 157.5)){
				goFrontLeft();
			}
			else if((ctlAngle >= 157.5) && (ctlAngle < 202.5)){
				goLeft();
			}
			else if((ctlAngle >= 202.5) && (ctlAngle < 247.5)){
				goBackLeft();
			}
			else if((ctlAngle >= 247.5) && (ctlAngle < 292.5)){
				goBack();
			}
			else{ //((ctlAngle >= 292.5) || (ctlAngle < 337.5))
				goBackRight();
			}
		}
	}
	else{
		updateSpeed(ctlPowerFromYaw);

		if(yaw - 1500 > 0){
			rotateRight();
		}
		else{
			rotateLeft();
		}	
	}
}

void inline SerialOpen(uint32_t baud);


void setup() {
  SerialOpen(SERIAL0_COM_SPEED); 

	#if DEUBUG_ON
  exSerial.begin(SERIAL1_COM_SPEED); 
  #endif

  while (!Serial) {
  }
  
  //blinkLED(STATUS_LED, 5, 100);

  deviceTime = millis();

 	motor1Speed = 0;
  motor2Speed = 0;
  motor3Speed = 0;
  motor4Speed = 0;

	sei();
} 

void loop() {
	unsigned long now = millis();


	if(now - hbTime >= STATUS_HB_DURATION){
    hbTime = now;

		logPrint("h");
		logPrintln(SerialAvailable(0));
  }

	serialCom();
	setThrottle();

  if(now - deviceTime >= STATUS_DEVICE_STICK_DURATION){
		if (failsafeCnt > 6) { 								
			for(uint8_t i=0; i < RC_CHANS; i++)
				serialRcValue[i] = 1500;
		}

		failsafeCnt++;

	
    deviceTime = now;

    for(uint8_t channelIdx = 0; channelIdx < RC_CHANS; channelIdx++){
  		rcData[channelIdx] = serialRcValue[channelIdx];
    }

		writeMotors();
  }

	delay(2);
}
