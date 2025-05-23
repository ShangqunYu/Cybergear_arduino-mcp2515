#include "CyberGearCAN.h"
#include <mcp2515.h>
CyberGearCAN gear(10);
float speed;

void setup() {
  Serial.begin(115200);
  gear.begin();  // default 1 Mb/s
  Serial.println("Enabling motor ...");
  gear.motorEnable(127);
  delay(100);
  Serial.println("Setting mode 2...");
//  gear.setSpeed(5, 5.0, 10.0);  // id=5, speed=50 rpm, limitCur=10 A
  gear.setMode(127,2);
  uint8_t buf[8];
  uint8_t len = 0;
  uint32_t canId = 0;
  gear.receiveFrame(buf, len, canId);
  gear.setSpeed(127, -10, 0);
  speed = -20.0;
}

void loop() {
//  float pos, vel;
//  if (gear.getState(5, pos, vel)) {
//    Serial.print("Pos: "); Serial.print(pos);
//    Serial.print("°  Vel: "); Serial.print(vel);
//    Serial.println(" rpm");
//  }
//  speed = speed + 0.1;
//  if (speed >= 20.0){
//    speed = -20.0;
//    }
  uint8_t buf[8];
  uint8_t len = 0;
  uint32_t canId = 0;
  gear.setSpeed(127, speed, 0);
  gear.receiveFrame(buf, len, canId);
  delay(200);
  
}
