#include <math.h>

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
#include "/Users/Nicholas/Documents/Arduino/libraries/mavlink/v1.0/ardupilotmega/mavlink.h"


#define MIN_DISTANCE 12
#define MAX_DISTANCE 24

#define RADIO_SERIAL Serial2
#define COPTER_SERIAL Serial1

#define ROTOR_GUIDED_MODE 4
#define ROTOR_LAND_MODE 9

#define MAV_COMP_ID_PRIMARY 1

int requested_stream_ = 0;
int trigPins[] = {50,48,46};
int echoPins[] = {51,49,47};
int numSensors = 1;
int sysid = 252;
int compid = 1;
int target_sysid = 1;
int target_compid = MAV_COMP_ID_PRIMARY;

void setup() {
    Serial.begin(57600);
    COPTER_SERIAL.begin(57600);
    RADIO_SERIAL.begin(57600);
}
void sendArm() {
    sendAction(MAV_CMD_COMPONENT_ARM_DISARM, 1.0, 0, 0, 0, 0, 0, 0);
}

void requestStream() {
    mavlink_message_t msg;
    mavlink_msg_request_data_stream_pack(sysid, compid, &msg, target_sysid, target_compid, MAV_DATA_STREAM_POSITION, 2, 1);
    sendMavlinkMessage(&msg);
}

void sendMavMode(int mode) {
    mavlink_message_t msg;
    mavlink_msg_set_mode_pack(sysid, compid, &msg, target_sysid, 1, mode);
    sendMavlinkMessage(&msg);
}

void sendMove(float x, float y) {
    mavlink_message_t msg;
    mavlink_msg_mission_item_pack(sysid, compid, &msg, target_sysid, target_compid,
    0, MAV_FRAME_LOCAL_NED, MAV_CMD_NAV_WAYPOINT, 1, 1,
    0, 0, 0, 0,
    x, y, 0);
    sendMavlinkMessage(&msg);
}

void sendAction(int action, float param1, float param2, float param3, float param4, float param5, float param6, float param7) {
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(sysid, compid, &msg, target_sysid, target_compid, action, 1, param1, param2, param3, param4, param5, param6, param7);
    sendMavlinkMessage(&msg);
}

void sendMavlinkMessage(mavlink_message_t* msg) {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    COPTER_SERIAL.write(buf, len);
}

void loop() {
    radio_receive();

    //sendMavMode(ROTOR_GUIDED_MODE);
    /*float dist=0;
    for(int i = 0;
    i< numSensors;
    i++) {
        dist = getDistance(trigPins[i], echoPins[i]);
        if (dist < MAX_DISTANCE && dist > MIN_DISTANCE) { //tinker/change
            //printDist(i, dist);
            sendArm();
            //sendMove(0, 4);
            //sendMavMode(ROTOR_GUIDED_MODE);
            //delay(10);
            //sendMove(0, 4);
        }
        delay(50);
    }*/
    
    //sendArm();
    delay(1000);
}

void printDist(int sensor, float distance) {
    int len = 0;
    char buffer[50];
    len = sprintf(buffer,"Sensor %i : %i", sensor, (int)distance);
    for(int l = 0;l < len;l ++) 
        Serial.print(buffer[l]);
    
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

float microsecondsToInches(float microseconds) {
    // 73.746 microseconds per inch (i.e. sound travels at 1130 feet per
    // second).
    // divide by 2 to get the distance of the obstacle.
    return microseconds / 74.0f / 2.0f;
}

void comm_receive() {
    mavlink_message_t msg;
    mavlink_status_t status;
    memset(&msg, 0, sizeof(mavlink_message_t));
    memset(&status, 0, sizeof(mavlink_status_t));
    char buffer[256];
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    while(COPTER_SERIAL.available() > 0) {
        uint8_t c = COPTER_SERIAL.read();
        // Try to get a new message
        if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
            // Handle message
            switch(msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT:
                  break;
                case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                  //mavlink_msg_global_position_int_get_lon(&msg) / 1.0E7f;
                  //mavlink_msg_global_position_int_get_lat(&msg) / 1.0E7f;
                  break;
                case MAVLINK_MSG_ID_COMMAND_LONG:
                  break;
                default:
                  break;
            }
        }
    }
}

void radio_receive() {
    while(RADIO_SERIAL.available() > 0) {
        uint8_t c = RADIO_SERIAL.read();
        COPTER_SERIAL.write(c);
    }
}
