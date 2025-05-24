#ifndef CYBERGEARCAN_H
#define CYBERGEARCAN_H

#define MAX_MOTORS 10

#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>
#include <string.h>

struct MotorState {
  float angle;        // in degree from -716 to 716 / -4π ~ 4π
  float speed;        // in rpm    
  float torque;       // in Nm
  float temp;    // in °C
  bool axisError;     // true if there’s an error
  uint8_t modeStatus; // e.g. 0=idle,1=running,2=fault,…
};


class CyberGearCAN {
public:
  // --------------------------------------------------------------------------
  // Constructor takes the MCP2515 CS pin
  CyberGearCAN(uint8_t csPin = 10);

  // initialize SPI & MCP2515 (call in setup)
  void begin(CAN_SPEED canSpeed = CAN_1000KBPS,
            CAN_CLOCK mcpClock = MCP_8MHZ);
  bool setMotorIDs(const uint8_t* ids, uint8_t count);
  void initMotors(uint8_t mode);
  // high-level API
  void motorEnable(uint8_t id = 127);
  void setMode(uint8_t id, uint8_t mode);
  void setAngle(uint8_t id, float angleDeg, float speedRpm, float limitCurA);
  void setSpeed(uint8_t id, float speedRpm, float limitCurA);
  void setTorque(uint8_t id, float torqueNm);
  void impedanceControl(uint8_t id,
                        float posDeg, float velRpm,
                        float tffNm, float kp, float kd);
  void motorEstop(uint8_t id = 127);
  void setZeroPosition(uint8_t id = 127);
  void clearError(uint8_t id = 127);
  // bool    setId(uint8_t oldId, uint8_t newId);
  // void    initConfig(uint8_t id = 127);

  // lower-level property read/write
  void writeProperty(uint8_t id, uint16_t index,
                     const char* type, uint32_t data);
  // float readProperty(uint8_t id, uint16_t index,
  //                    const char* type);

  // status queries
  // void  getId(uint8_t id = 127);
  void  getState(uint8_t id);
  // bool  getVolCur(uint8_t id, float& outVolts, float& outAmps);
  bool receiveFrame();
private:
  MCP2515   _mcp;
  uint8_t   _masterId;
  uint8_t   _mcuId[8];
  // physical limits from Python lib:
  const float P_MIN=-12.5,  P_MAX=12.5;
  const float V_MIN=-30.0,  V_MAX=30.0;
  const float KP_MIN=0.0,   KP_MAX=500.0;
  const float KD_MIN=0.0,   KD_MAX=5.0;
  const float T_MIN=-12.0,  T_MAX=12.0;
  const float TORQUE_CONST = T_MAX / 27.0;
  const float RAD_DEG = 180.0/PI;
  const float DEG_RAD = PI/180.0;
  const float RAD_S_RMIN = 30.0/PI;
  const float RMIN_RAD_S = PI/30.0;

  uint8_t _num_motor;
  uint8_t _motorIDs[MAX_MOTORS];
  MotorState _motorStates[MAX_MOTORS];

  // internal send/receive
  void sendCommand(uint8_t id,
                   uint8_t conType,
                   uint8_t conData[2],
                   uint8_t payload[8]);
  

  // packing/unpacking helpers
  uint32_t floatToUint(float x, float xMin, float xMax, uint8_t bits);
  float    uintToFloat(uint32_t x, float xMin, float xMax, uint8_t bits);
  int8_t findMotorIndex(uint8_t id);
};

#endif
