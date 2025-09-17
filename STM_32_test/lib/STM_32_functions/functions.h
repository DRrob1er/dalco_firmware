#include <Arduino.h>
#include <eXoCAN.h>
#ifndef FUNCTIONS_H


#define FUNCTIONS_H

int test_func(int a, int b);
void sendNMTCommand(uint8_t cs, uint8_t nodeId);
// void setup_motor_mode(int nodeId, int config);
// void writeControlword(int nodeId, uint16_t type);
// void desired_motor_speed_SDO(int nodeId, int speed);
void enableMotor(int nodeId);
void sendSDOReadRequest(uint8_t nodeId, uint16_t index, uint8_t subIndex);
void SDO_write(int nodeId,  int down_byte,  int IDX, int IDX_sub, int32_t data);
void PDO_write_int32(int nodeId, int PDO_Id, int32_t data_pos, int32_t data_vel);
void CAN_filters();
void PDO_write_int16(int nodeId, int PDO_Id, int16_t data);
int getState(const uint8_t bytes[2]);
int getHomingState(const uint8_t bytes[2]);
void standbyMotor(int nodeId);
void reset_error(int motor_id);
int calculatePositionInPulses(double desiredAngle, int pulsesPerFullRotation, int homingPulsesOffset, double transmissionRatio);
double calculatePositionInDegrees(int pulsesPerFullRotation, int homingPulsesOffset, double transmissionRatio, int currentPositionInPulses);

extern eXoCAN myCAN;

#endif