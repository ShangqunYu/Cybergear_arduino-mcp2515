#include "CyberGearCAN.h"
#include <mcp2515.h>
CyberGearCAN gear(10);
float speed;

void setup() {
  Serial.begin(115200);
  delay(500);
  uint8_t myIDs[] = {127};
  gear.setMotorIDs(myIDs, sizeof(myIDs));
  gear.begin();  // default 1 Mb/s
  delay(100);
  gear.initMotors(2);
  Serial.println("motor enabled ...");
  // gear.motorEnable(127);

//  gear.setSpeed(5, 5.0, 10.0);  // id=5, speed=50 rpm, limitCur=10 A
  // gear.setMode(127,2);

//  gear.receiveFrame();
  gear.setSpeed(127, -15, 0);
//  delay(3000);
  
//  gear.motorEstop(127);
//  gear.setZeroPosition(127);
//  gear.motorEnable(127);
//  gear.setSpeed(127, 10.0, 0);
}

void loop() {
  float pos, vel;
//  if (gear.getState(5, pos, vel)) {
//    Serial.print("Pos: "); Serial.print(pos);
//    Serial.print("°  Vel: "); Serial.print(vel);
//    Serial.println(" rpm");
//  }
//  speed = speed + 0.1;
//  if (speed >= 20.0){
//    speed = -20.0;
//    }
  gear.setSpeed(127, -15, 0);
  gear.receiveFrame();
// gear.getState(127, pos, vel);
//  Serial.print("Pos: "); Serial.println(pos);
//  Serial.print("Vel: "); Serial.println(vel);
  delay(200);
  
}
