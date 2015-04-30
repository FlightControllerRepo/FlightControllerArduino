#include <math.h>

//defines for floating point math functions
#define     cosf   cos
#define     sinf   sin
#define     tanf   tan
#define     fabsf   fabs
#define     fmodf   fmod
#define     cbrtf   cbrt
#define     hypotf   hypot
#define     squaref   square
#define     floorf   floor
#define     ceilf   ceil
#define     frexpf   frexp
#define     ldexpf   ldexp
#define     expf   exp
#define     coshf   cosh
#define     sinhf   sinh
#define     tanhf   tanh
#define     acosf   acos
#define     asinf   asin
#define     atanf   atan
#define     atan2f   atan2
#define     logf   log
#define     log10f   log10
#define     powf   pow
#define     isnanf   isnan
#define     isinff   isinf
#define     isfinitef   isfinite
#define     copysignf   copysign
#define     signbitf   signbit
#define     fdimf   fdim
#define     fmaf   fma
#define     fmaxf   fmax
#define     fminf   fmin
#define     truncf   trunc
#define     roundf   round
#define     lroundf   lround
#define     lrintf   lrint
#define     sqrtf    sqrt
#include "~Arduino/libraries/mavlink/v1.0/ardupilotmega/mavlink.h"

//This defines the serial type. The Radio serial writes/reads from
//the radio, while the copter serial writes/reads from the pixhawk.
//See the readme for more information
#define RADIO_SERIAL Serial2
#define COPTER_SERIAL Serial1


//These define the thresholds for our ultrasonic sensors
#define MIN_DISTANCE 12
#define MAX_DISTANCE 24

//our sensor functions, each one preforms an action, and is passed in the distance
//from the sensor
void sensor_front(float);
void sensor_back(float);
void sensor_left(float);
void sensor_right(float);

//here are the pins that for the ultrasonic sensors, there are 2
//pins for activiting the sensor, the echo end trig pins.
int trigPins[] = {50,48,46,44};
int echoPins[] = {51,49,47,45};
//these are the functions called when a sensor gets triggered. You can add more
//sensors by adding to the trigPins/echoPins array, and then adding a handler
//for that sensor here
void (* sensors[])(float) = {sensor_left, sensor_right, sensor_front, sensor_back};
int numSensors = 4;

//mavlink specific constants
int sysid = 252;
int compid = 1;
int target_sysid = 1;
int target_compid = MAV_COMP_ID_PRIMARY;

#define ROTOR_GUIDED_MODE 4
#define ROTOR_LAND_MODE 9

#define MAV_COMP_ID_PRIMARY 1

void setup() {
    //initilize serials
    Serial.begin(57600);
    COPTER_SERIAL.begin(57600);
    RADIO_SERIAL.begin(57600);
}

//function to send an arm, useful for testing
void sendArm() {
    sendAction(MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 0, 0, 0, 0, 0, 0);
}

//resuts a data stream from the copter, this particular function is hardcoded
//to receive GPS coords
void requestStream() {
    mavlink_message_t msg;
    mavlink_msg_request_data_stream_pack(sysid, compid, &msg, target_sysid, target_compid, MAV_DATA_STREAM_POSITION, 2, 1);
    sendMavlinkMessage(&msg);
}

//sets the mode of the copter, such as Land, Guided, etc
void sendMavMode(int mode) {
    mavlink_message_t msg;
    mavlink_msg_set_mode_pack(sysid, compid, &msg, target_sysid, 1, mode);
    sendMavlinkMessage(&msg);
}

//send a move command. The copter must be in guieded or loiter mode for this to work
//x and y are local coordinates
void sendMove(float x, float y) {
    mavlink_message_t msg;
    mavlink_msg_mission_item_pack(sysid, compid, &msg, target_sysid, target_compid,
    0, MAV_FRAME_LOCAL_NED, MAV_CMD_NAV_WAYPOINT, 1, 1,
    0, 0, 0, 0,
    x, y, 0);
    sendMavlinkMessage(&msg);
}

//sends an action to the copter, sendArm uses this
void sendAction(int action, float param1, float param2, float param3, float param4, float param5, float param6, float param7) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(sysid, compid, &msg, target_sysid, target_compid, action, 1, param1, param2, param3, param4, param5, param6, param7);
    sendMavlinkMessage(&msg);
}

//send status text back through the radio, to be received on the phone.
void sendStatusText(char* txt) {
  char buffer[50];
  memset(buffer, 0, 50);
  strcpy(buffer, txt);
  
  mavlink_message_t msg; 
  mavlink_msg_statustext_pack(sysid, compid, &msg, 0, buffer);
  sendMavlinkMessageRadio(&msg);
}

//sends a mavlink message through the radio
void sendMavlinkMessageRadio(mavlink_message_t* msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    RADIO_SERIAL.write(buf, len);
}

//sends a mavlink message through to the copter
void sendMavlinkMessage(mavlink_message_t* msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    COPTER_SERIAL.write(buf, len);
}

//our sensor handles
void sensor_left(float data) {
    sendStatusText("Collision on left");
    sendMove(4, 0);
} 

void sensor_right(float data) {
    sendStatusText("Collision on right");
    sendMove(-4, 0);
} 

void sensor_front(float data) {
    sendStatusText("Collision on front");
    sendMove(0, -4);
} 

void sensor_back(float data) {
    sendStatusText("Collision on back");
    sendMove(4, 0);
} 
//end sensor handles

void loop() {
    radio_receive(); //passes data from the pixhawk to the radio, and from the 
    //radio to the pixhawk

    //activates the sensors 
    float dist = 0;
    for(int i = 0;i< numSensors;i++) {
        dist = getDistance(trigPins[i], echoPins[i]);
        if (dist < MAX_DISTANCE && dist > MIN_DISTANCE) { 
            //printDist(i, dist);
            sensors[i](dist);
        }
        delay(50);
    }
    
}

//utility function to print our the serial information
void printDist(int sensor, float distance) {
    int len = 0;
    char buffer[50];
    len = sprintf(buffer,"Sensor %i : %i", sensor, (int)distance);
    for(int l = 0;l < len;l ++) 
        Serial.print(buffer[l]);
    
    Serial.println();
}

//gets the distance readings from an ultrasonic sensor
float getDistance(int trigPin, int echoPin) {
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

//utility fuction to conver microsecond readings to inches, for the ultrasonic
//sensors
float microsecondsToInches(float microseconds) {
    // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
    // second).
    // divide by 2 to get the distance of the obstacle.
    return microseconds / 74.0f / 2.0f;
}

//Our function that serializes messages between the radio and pixhawk, 
//and passes them through.
void radio_receive() {
    mavlink_message_t msg;
    mavlink_status_t status;
    memset(&msg, 0, sizeof(mavlink_message_t));
    memset(&status, 0, sizeof(mavlink_status_t));
  
    while(RADIO_SERIAL.available() > 0) {
        uint8_t c = RADIO_SERIAL.read(); //read from radio
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
          uint8_t buf[MAVLINK_MAX_PACKET_LEN];
          uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
          COPTER_SERIAL.write(buf, len); //write out message to pixhawk
          memset(&msg, 0, sizeof(mavlink_message_t));
          memset(&status, 0, sizeof(mavlink_status_t));
        }
    }
    
    memset(&msg, 0, sizeof(mavlink_message_t));
    memset(&status, 0, sizeof(mavlink_status_t));
    while (COPTER_SERIAL.available() > 0) {
        uint8_t c = COPTER_SERIAL.read(); //read from pixhawk
        if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
          uint8_t buf[MAVLINK_MAX_PACKET_LEN];
          uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
          RADIO_SERIAL.write(buf, len); //write out message to radio
          
          memset(&msg, 0, sizeof(mavlink_message_t));
          memset(&status, 0, sizeof(mavlink_status_t));
        }
    }
}
