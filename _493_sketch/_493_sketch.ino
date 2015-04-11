#include <math.h>

#define 	cosf   cos
#define 	sinf   sin
#define 	tanf   tan
#define 	fabsf   fabs
#define 	fmodf   fmod
#define 	cbrtf   cbrt
#define 	hypotf   hypot
#define 	squaref   square
#define 	floorf   floor
#define 	ceilf   ceil
#define 	frexpf   frexp
#define 	ldexpf   ldexp
#define 	expf   exp
#define 	coshf   cosh
#define 	sinhf   sinh
#define 	tanhf   tanh
#define 	acosf   acos
#define 	asinf   asin
#define 	atanf   atan
#define 	atan2f   atan2
#define 	logf   log
#define 	log10f   log10
#define 	powf   pow
#define 	isnanf   isnan
#define 	isinff   isinf
#define 	isfinitef   isfinite
#define 	copysignf   copysign
#define 	signbitf   signbit
#define 	fdimf   fdim
#define 	fmaf   fma
#define 	fmaxf   fmax
#define 	fminf   fmin
#define 	truncf   trunc
#define 	roundf   round
#define 	lroundf   lround
#define 	lrintf   lrint
#define         sqrtf    sqrt
#define MAV_COMP_ID_PRIMARY 1

#include "/Users/mehulgoyal/Documents/cs493/arduino/libraries/mavlink/v1.0/ardupilotmega/mavlink.h"

int trigPins[] = {50,48,46};
int echoPins[] = {51,49,47};
int numSensors = 3;
int sysid = 252;
int compid = 1;
int target_sysid = 1;
int target_compid = MAV_COMP_ID_PRIMARY;

void setup() {
  Serial.begin(9600);
  Serial1.begin(57600);
  
  for (int i = 0; i < 10; i++) {// tinker/change
    sendArm();
  }
}

void sendArm() {
  sendMavlinkMessage(MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 0, 0, 0, 0, 0, 0);
}

void sendMavlinkMessage(int action, float param1, float param2, float param3, float param4, float param5, float param6, float param7) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_command_long_pack(sysid, compid, &msg, target_sysid, target_compid, action, 1, param1, param2, param3, param4, param5, param6, param7);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  Serial1.write(buf,len);
}

void loop() {
  comm_receive();
  float dist=0;
  for(int i = 0; i< numSensors; i++) { 
      dist = getDistance(trigPins[i], echoPins[i]);
      if (dist < 15) { //tinker/change
        // turn 180 degrees (relative to current yaw) at 5 degrees per second, clockwise
        sendMavlinkMessage(MAV_CMD_CONDITION_YAW, 180.0, 5.0, 1, 1, 0, 0, 0);
      }
      delay(50);
    }
    delay(100);
}

void printDist(int sensor, float distance) {
  int len = 0;
  char buffer[50];
  len = sprintf(buffer,"Sensor %i : %i", sensor, (int)distance);
  for(int l= 0; l<len; l++) {
    Serial.print(buffer[l]);
  }
  Serial.println();
}

float getDistance(int trigPin,int echoPin) {
  float distance, inches;
  // Clean reception for 2 microseconds
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // send signal for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  //receive signal from HIGh pulse
  pinMode(echoPin, INPUT);
  distance = pulseIn(echoPin, HIGH);

  // convert the time into a distance
  inches = microsecondsToInches(distance);
  return inches;
}

float microsecondsToInches(float microseconds)
{
  // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
  // second). 
  // divide by 2 to get the distance of the obstacle.
  return microseconds / 74.0f / 2.0f;
}

void comm_receive() {
 
       mavlink_message_t msg;
	mavlink_status_t status;
  
	while(Serial1.available() > 0 ) 
	{
		uint8_t c = Serial1.read();
		// Try to get a new message
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			// Handle message
                       
			switch(msg.msgid)
			{
			        case MAVLINK_MSG_ID_HEARTBEAT:
			        {
                                    Serial.print(msg.msgid); 
                                   Serial.println("heartbeat");  
				  // E.g. read GCS heartbeat and go into
                                  // comm lost mode if timer times out
                                  break;
			        }
			        
			case MAVLINK_MSG_ID_COMMAND_LONG:
				// EXECUTE ACTION
                                Serial.print(msg.msgid); 
                                Serial.println("MESSAGE LONG"); 
				break;
			default:
				//Do nothing
                                Serial.print("default case ");
                                Serial.print(msg.msgid);
                                Serial.print(" ");                               
                                Serial.println(MAVLINK_MSG_ID_COMMAND_LONG);
				break;
			}
		}
 
		// And get the next one
	}
}
