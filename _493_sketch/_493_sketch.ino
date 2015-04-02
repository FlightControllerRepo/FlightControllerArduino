#include <mavlink.h>

int trigPins[] = {50,48,46};
int echoPins[] = {51,49,47};
int numSensors = 3;
int system_type = MAV_QUADROTOR;
int autopilot_type = MAV_AUTOPILOT_PIXHAWK;
int sysid = -1; // fix
int compid = -1; // fix
int target_sysid = -1; // fix
int target_compid = -1; // fix

void setup() {
  Serial.begin(9600);
}

void sendHeartbeat() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_heartbeat_pack(sysid, compid, &msg, system_type, autopilot_type);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // send message (uart ?)
}

void sendMavlinkMessage(int action) {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_action_pack(sysid, compid, &msg, target_sysid, target_compid, action);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
   // send message (uart ?)
}

void loop() {
  float dist=0;
  for(int i = 0; i< numSensors; i++) { 
      dist = getDistance(trigPins[i], echoPins[i]);
      printDist(i, dist);
      //delay for each individual sensor
      delay(50);
    }
    Serial.println();
    Serial.println();
    Serial.println();
    delay(1000);
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
