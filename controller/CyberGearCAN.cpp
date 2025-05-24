#include "CyberGearCAN.h"
// ——— Constructor ———————————————————————————————————————————————————————
CyberGearCAN::CyberGearCAN(uint8_t csPin)
  : _mcp(csPin), _masterId(0) { }

// ——— begin() ———————————————————————————————————————————————————————————
void CyberGearCAN::begin(CAN_SPEED canSpeed, CAN_CLOCK mcpClock) {
  SPI.begin();
  _mcp.reset();
  _mcp.setBitrate(canSpeed, mcpClock);
  _mcp.setNormalMode();
}

// ——— Initalize all the Motors() ————————————————————————————————————————
void CyberGearCAN::initMotors(uint8_t mode){
  for (int i; i < _num_motor; i++){
    motorEnable(_motorIDs[i]);
    setMode(_motorIDs[i], mode);
    setZeroPosition(_motorIDs[i]);
  }
}

// ——— Store every motors id and state() —————————————————————————————————
bool CyberGearCAN::setMotorIDs(const uint8_t* ids, uint8_t count){
  if (count > MAX_MOTORS) return false;
  _num_motor = count;
  Serial.print("_num_motor: ");  Serial.println(_num_motor);
  for (uint8_t i = 0; i < _num_motor; ++i) {
    _motorIDs[i]    = ids[i];
    _motorStates[i] = MotorState();  
  }
  return true;  
}

// linear search to map ID → index
int8_t CyberGearCAN::findMotorIndex(uint8_t id) {
  for (uint8_t i = 0; i < _num_motor; ++i){
    if (_motorIDs[i] == id) return i;
  }
  Serial.println("can't find the motor index, will just use the 0 index to prevent seg fault!!!");
  return 0;
}

// ——— sendCommand ——————————————————————————————————————————————————————
void CyberGearCAN::sendCommand(uint8_t id,
                               uint8_t conType,
                               uint8_t conData[2],
                               uint8_t payload[8]) {
  struct can_frame frame;
  // build a 29-bit ID:    Bit 28 ~ 24 |	Bit 23 ~ 8  | Bit 7 ~ 0
  //                Communication type |	Data area 2	| Target addres
  uint32_t effId = (uint32_t)conType << 24 | (uint32_t)conData[1] << 16 | (uint32_t)conData[2] << 8 | (uint32_t)id;
  // indicating it's a extended frame
  frame.can_id = effId | CAN_EFF_FLAG;
  frame.can_dlc = 8;
  for(uint8_t i=0;i<8;i++) {
    frame.data[i] = payload[i];
  }
  _mcp.sendMessage(MCP2515::TXB0, &frame);
}

// ——— receiveCommand  ————————————————————————————————————————
bool CyberGearCAN::receiveFrame() {
  struct can_frame frame;
  if (_mcp.readMessage(MCP2515::RXB0, &frame) == MCP2515::ERROR_OK) {
    // get the 29 bit ID
    uint32_t canId = frame.can_id & CAN_EFF_MASK;
    uint32_t conMode = (canId >> 24) & 0x1F;
    uint32_t motorId = (canId >> 8) & 0xff;
    uint8_t motorIndex = findMotorIndex(motorId);
    //Byte 0 ~ 1: Current angle [0 ~ 65535] corresponds to (-4π ~ 4π)
    uint16_t angle = (frame.data[0] << 8) | frame.data[1];
    _motorStates[motorIndex].angle  =  uintToFloat(angle, P_MIN, P_MAX, 16) * RAD_DEG;
    //Byte 2 ~ 3: Current angular velocity [0 ~ 65535] corresponds to (-30rad/s ~ 30rad/s)
    uint16_t speed = (frame.data[2] << 8) | frame.data[3];
    _motorStates[motorIndex].speed  =  uintToFloat(speed, V_MIN, V_MAX, 16) * RAD_S_RMIN;
    //Byte 4 ~ 5: Current torque [0 ~ 65535] corresponds to (-12Nm ~ 12Nm)
    uint16_t torque = (frame.data[4] << 8) | frame.data[5];
    _motorStates[motorIndex].torque = uintToFloat(torque, T_MIN, T_MAX, 16);
    //Byte 6 ~ 7: Current temperature: Temp (degrees Celsius) )*10
    _motorStates[motorIndex].temp   = ((frame.data[6] << 8) | frame.data[7]) * 0.1;
    //Bit 22 ~ 23 of 29 bit ID: mode status: 0: Reset mode [reset] 1: Cali mode [Calibration] 2: Motor mode [Run]
    _motorStates[motorIndex].modeStatus = (canId >> 22) & 0x3;
    //Bit 21 ~ 16 of 29 bit ID: Fault information (0 - No, 1 - Yes)
    _motorStates[motorIndex].axisError =  (canId >> 16) & 0x3f;
    Serial.print("angle: ");  Serial.println(_motorStates[motorIndex].angle);
    Serial.print("speed: ");  Serial.println(_motorStates[motorIndex].speed);
    Serial.print("torque: "); Serial.println(_motorStates[motorIndex].torque);    
    Serial.print("temp: ");   Serial.println(_motorStates[motorIndex].temp);
    Serial.print("modeStatus: ");   Serial.println(_motorStates[motorIndex].modeStatus);
    Serial.print("axisError: ");   Serial.println(_motorStates[motorIndex].axisError);
    Serial.print("overcurrent: ");  Serial.println(canId >> 17 & 0x1);
    Serial.print("over temperature: ");  Serial.println(canId >> 18 & 0x1);
    Serial.print("Magnetic encoding failure: ");  Serial.println(canId >> 19 & 0x1);
    Serial.print("HALL encoding failure: ");  Serial.println(canId >> 20 & 0x1);
    Serial.print("not calibrated: ");  Serial.println(canId >> 21 & 0x1);
    Serial.print("overcurrent: ");  Serial.println(canId >> 17 & 0x1);
    // uint8_t len   = frame.can_dlc;
    // memcpy(buf, frame.data, len);
    return true;
  }
  return false;
}

// ——— format converters —————————————————————————————————————————————
uint32_t CyberGearCAN::floatToUint(float x, float xMin, float xMax, uint8_t bits) {
  if      (x < xMin) x = xMin;
  else if (x > xMax) x = xMax;
  float span = xMax - xMin;
  return (uint32_t)( (x - xMin) * ((1UL<<bits)-1) / span );
}

float CyberGearCAN::uintToFloat(uint32_t x, float xMin, float xMax, uint8_t bits) {
  uint32_t span = (1UL<<bits) - 1;
  if (x > span) x = span;
  float fspan = xMax - xMin;
  return xMin + ( (float)x / (float)span ) * fspan;
}

// ——— Enable the motor ——————————————————————————————————————————————————
void CyberGearCAN::motorEnable(uint8_t id) {
  uint8_t conData[2] = { 0, 0 };
  uint8_t data8[8] = {0};
  sendCommand(id, 3, conData, data8);
  // everytime calling motor enable, the motor will need some time to enable, so need to delay. 
  delay(50);
}

// ——— 0: Operation control  1: Position  2: Speed  3: Current  ——————————————————————————————————————————————————
void CyberGearCAN::setMode(uint8_t id, uint8_t mode) {
  writeProperty(id, 0x7005, "u", uint32_t(mode));
}

void CyberGearCAN::setAngle(uint8_t id, float angleDeg, float speedRpm, float limCur) {
  // motorEnable(id);
  setMode(id, 1);
  uint32_t cdata;
  memcpy(&cdata, &limCur, sizeof(limCur));
  writeProperty(id, 0x7018, "f", cdata);
  uint32_t sdata;
  float radPerS =  speedRpm * RMIN_RAD_S;
  memcpy(&sdata, &radPerS, sizeof(radPerS));
  writeProperty(id, 0x7017, "f", sdata);
  uint32_t data;
  float rad = angleDeg * DEG_RAD;
  memcpy(&data, &rad, sizeof(rad));
  writeProperty(id, 0x7016, "f", data);
}

void CyberGearCAN::setSpeed(uint8_t id, float speedRpm, float limCur) {
  // motorEnable(id);
  setMode(id, 2);

  uint32_t data; // = floatToUint(speedRpm * RMIN_RAD_S, V_MIN, V_MAX, 32);
  float radPerS = speedRpm * RMIN_RAD_S;
  memcpy(&data, &radPerS, sizeof(radPerS));

  writeProperty(id, 0x700A, "f", data);
}

void CyberGearCAN::setTorque(uint8_t id, float torqueNm) {
  motorEnable(id);
  setMode(id, 3);
  // writeProperty(id, 0x7006, "f", torqueNm / TORQUE_CONST);
}

void CyberGearCAN::impedanceControl(uint8_t id,
                                    float posDeg, float velRpm,
                                    float tffNm, float kp, float kd) {
  motorEnable(id);
  setMode(id, 0);
  // cmd_data is 16-bit torque feed-forward, big-endian
  uint32_t tfu = floatToUint(tffNm, T_MIN, T_MAX, 16);
  uint8_t cmd[2] = { (uint8_t)(tfu & 0xFF), (uint8_t)((tfu >> 8) & 0xFF) };
  uint8_t d[8];
  uint32_t u;
  // pos
  u = floatToUint(posDeg * DEG_RAD, P_MIN, P_MAX, 16);
  d[0] = (u>>8)&0xFF; d[1] = u&0xFF;
  // vel
  u = floatToUint(velRpm * RMIN_RAD_S, V_MIN, V_MAX, 16);
  d[2] = (u>>8)&0xFF; d[3] = u&0xFF;
  // kp
  u = floatToUint(kp, KP_MIN, KP_MAX, 16);
  d[4] = (u>>8)&0xFF; d[5] = u&0xFF;
  // kd
  u = floatToUint(kd, KD_MIN, KD_MAX, 16);
  d[6] = (u>>8)&0xFF; d[7] = u&0xFF;

  sendCommand(id, 1, cmd, d);
}

void CyberGearCAN::motorEstop(uint8_t id) {
  uint8_t cmd[2] = { 0, 0 };
  uint8_t d[8]={0};
  sendCommand(id, 4, cmd, d);
}

void CyberGearCAN::setZeroPosition(uint8_t id) {
  // do an estop, then send zero idx
  // motorEstop(id);
  uint8_t conData[2]={0,0}, d[8]={0};
  d[0]=0x01;
  sendCommand(id, 6, conData, d);
  // re-enable if was vel mode etc can be added
}

void CyberGearCAN::clearError(uint8_t id) {
  uint8_t cmd[2]={0,0}, d[8]={0};
  d[0]=0x01;
  sendCommand(id, 4, cmd, d);
}

// bool CyberGearCAN::setId(uint8_t oldId, uint8_t newId) {
//   // fetch MCU ID first
//   getId(oldId);
//   // implement getId→_mcuIdBytes[8] storage in class if needed...
//   // then:
//   motorEstop(oldId);
//   delay(100);
//   // assume you stored 8-byte MCU_ID in a member uint8_t _mcuId[8]
//   uint8_t cmd[2]={0, newId};
//   uint8_t d[8];
//   memcpy(d, _mcuId, 8);
//   sendCommand(oldId, 7, cmd, d);
//   delay(100);
//   return true; // no error checking here
// }

// void CyberGearCAN::initConfig(uint8_t id) {
//   uint8_t cmd[2]={0,3}, d[8]={0};
//   sendCommand(id, 8, cmd, d);
//   delay(3000);
//   setId(127, id);
// }

// ——— writeProperty / readProperty —————————————————————————————————————
void CyberGearCAN::writeProperty(uint8_t id, uint16_t index,
                                 const char* type, uint32_t data ) {
  uint8_t conType[2] = { 0, 0 }; // host can ID
  uint8_t d[8] = {0};
  d[0] = (uint8_t)(index & 0xFF);
  d[1] = (uint8_t)(index >> 8);
  // index low / high


  // pack the payload
  if (strcmp(type,"f")==0) {
    d[4] =  data        & 0xFF;  
    d[5] = (data >>  8) & 0xFF;  
    d[6] = (data >> 16) & 0xFF;  
    d[7] = (data >> 24) & 0xFF;  
  } else {
    d[4] = data & 0xFF; 
  }
  sendCommand(id, 18, conType, d);
  receiveFrame();
}

// float CyberGearCAN::readProperty(uint8_t id, uint16_t index,
//                                  const char* type) {
//   uint8_t cmd[2]={0,0}, d[8]={0};
//   d[0] = index & 0xFF;
//   d[1] = (index>>8)&0xFF;
//   uint8_t mode = (index<0x7000) ? 9 : 17;
//   if (index<0x7000) {
//     const char* types[] = {"u8","s8","u16","s16","u32","s32","f"};
//     for(uint8_t i=0;i<7;i++)
//       if (strcmp(type,types[i])==0) d[2]=i;
//   }
//   sendCommand(id, mode, cmd, d);
//   // now wait for reply
//   uint8_t buf[8], dlc;
//   uint32_t fid;
//   unsigned long t0 = millis();
//   while(millis()-t0 < 500) {
//     if (receiveFrame(buf, dlc, fid) && (buf[0]==mode)) {
//       // payload starts at buf[1]
//       if (strcmp(type,"f")==0) {
//         union { float f; uint8_t b[4]; } ru;
//         for(int i=0;i<4;i++) ru.b[i]=buf[4+i];
//         return ru.f;
//       } else {
//         // integer type
//         uint32_t u=0, bits = 8*atoi(&type[1]);
//         for(int i=0;i<((bits+7)/8);i++){
//           u |= ((uint32_t)buf[4+i]) << (8*i);
//         }
//         return (float)u;
//       }
//     }
//   }
//   return 0.0; // timeout
// }

// ——— getId / getState / getVolCur —————————————————————————————————————
// void CyberGearCAN::getId(uint8_t id) {
//   // send same as set mode 0 but masterId=0xFD
//   _masterId = 0xFD;
//   uint8_t cmd[2]={0,0}, d[8]={0};
//   sendCommand(id, 0, cmd, d);
//   // then read back into _mcuId[8]
//   uint8_t buf[8], dlc; uint32_t fid;
//   unsigned long t0=millis();
//   while (millis()-t0<500) {
//     if (receiveFrame(buf, dlc, fid) && buf[0]==0) {
//       // buf[4..11] is MCU ID
//       for(int i=0;i<8;i++) _mcuId[i]=buf[4+i];
//       break;
//     }
//   }
//   _masterId = 0; // restore
// }

void CyberGearCAN::getState(uint8_t id) {
  // writeProperty(id, 0x7018, "f", uint32_t(27));
  uint8_t cmd[2] = { 0, 0 };
  uint8_t d[8]={0};
  sendCommand(id, 0x15, cmd, d);
  receiveFrame();
}

// bool CyberGearCAN::getVolCur(uint8_t id, float& volts, float& amps) {
//   float v = readProperty(id, 0x302B, "f");
//   float a = readProperty(id, 0x301E, "f");
//   if (v!=0 || a!=0) {
//     volts = v * 0.001f;
//     amps  = a;
//     return true;
//   }
//   return false;
// }
