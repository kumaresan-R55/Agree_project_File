#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <ModbusMaster.h>

void setupSteeringMotor(ModbusMaster& node, const String& label, const String& pr_mode);
void writeSteeringPosition(ModbusMaster& node, int32_t position);

void setupTractionMotor(ModbusMaster& node, const String& label, const String& pr_mode);
void setTractionVelocity(ModbusMaster& node, int16_t rpm);

void writeRegister(ModbusMaster& node, uint16_t reg, uint16_t val);
int32_t readEncoder(ModbusMaster &node);
void writeSteeringPosition(ModbusMaster& node, int32_t position);
void moveRelative(ModbusMaster& node, int32_t delta);
void stopAllTraction();
void startHomingAllPR0();
void pr0AbsoluteMove(ModbusMaster& node, long pos, const char* tag);

// ---- Extern globals (defined in .ino) ----
extern ModbusMaster nodes[8];

extern bool    homingDone[4];
extern int32_t basePosition[4];

extern const uint8_t steering_ids[4];
extern const char*  steering_tag[4];
// in motor_control.h
extern const uint8_t traction_ids[4];

extern const uint8_t motor_map[4];

extern long  HOME_ABS_POS[4];
extern const float SCALE_FACTOR;


#define HOME_TOL_PULSES       1500   // acceptable error band to consider "homed"
#endif
