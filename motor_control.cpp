#include "motor_control.h"

void writeRegister(ModbusMaster& node, uint16_t reg, uint16_t val) {
  uint8_t result = node.writeSingleRegister(reg, val);
  // Serial2.print("Reg 0x"); Serial2.print(reg, HEX);
  // Serial2.print(" = ");
  // if (reg == 0x620B) Serial2.print((int16_t)val);
  // else Serial2.print(val);
  // Serial2.print(" ➜ "); Serial2.println(result == node.ku8MBSuccess ? "✅ OK" : "❌ Fail");
  delay(10);
}
int32_t readEncoder(ModbusMaster& node) {
  uint8_t result = node.readHoldingRegisters(0x0B1C, 2);
  if (result == node.ku8MBSuccess) {
    return ((int32_t)node.getResponseBuffer(0) << 16) | node.getResponseBuffer(1);
  }
  return 0;
}
void setupSteeringMotor(ModbusMaster& node, const String& label, const String& pr_mode) {
  Serial2.print("🛠️ Setup: ");
  Serial2.print(label);
  Serial2.print(" (");
  Serial2.print(pr_mode);
  Serial2.println(")");

  writeRegister(node, 0x6002, 0x0040);  // Emergency stop
  delay(10);

  
  writeRegister(node, 0x6200, 0x0001);  // PR0 = absolute position mode
  writeRegister(node, 0x6204, 0x0032);  // PR0 accel
  writeRegister(node, 0x6205, 0x0032);  // PR0 decel
  writeRegister(node, 0x6203, 600);
}
void writeSteeringPosition(ModbusMaster& node, int32_t position) {
  Serial2.print("🎯 ABS ➜ ");
  Serial2.println(position);
  uint32_t pos = (uint32_t)position;
  writeRegister(node, 0x6201, (pos >> 16) & 0xFFFF);
  writeRegister(node, 0x6202, pos & 0xFFFF);
  writeRegister(node, 0x6002, 0x0010);
  delay(100);
}

void setupTractionMotor(ModbusMaster& node, const String& label, const String& pr_mode) {
  Serial2.print("🛠️ Setup: ");
  Serial2.print(label);
  Serial2.print(" (");
  Serial2.print(pr_mode);
  Serial2.println(")");

  // --- Safety pre-stop before setup ---
  writeRegister(node, 0x6002, 0x0040);  // Emergency stop
  delay(10);

  // --- Configure PR0 as velocity mode ---
  writeRegister(node, 0x6200, 0x0002);  // PR0 = Velocity mode
  writeRegister(node, 0x6204, 0x0032);  // PR0 acceleration (50)
  writeRegister(node, 0x6205, 0x0032);  // PR0 deceleration (50)
}
void moveRelative(ModbusMaster& node, int32_t delta) {
  Serial2.print("↪️ REL +");
  Serial2.println(delta);
  uint32_t rel = (uint32_t)delta;
  writeRegister(node, 0x6208, 0x0041);
  writeRegister(node, 0x6209, (rel >> 16) & 0xFFFF);
  writeRegister(node, 0x620A, rel & 0xFFFF);
  writeRegister(node, 0x620B, 600);
  writeRegister(node, 0x620C, 50);
  writeRegister(node, 0x620D, 50);
  writeRegister(node, 0x6002, 0x0011);              
}

/*void setTractionVelocity(ModbusMaster& node, int16_t rpm) {
  writeRegister(node, 0x620B, rpm);
  writeRegister(node, 0x6002, 0x0011);
}*/

void setTractionVelocity(ModbusMaster& node, int16_t rpm) {
  // --- PR0 velocity setup sequence ---
  writeRegister(node, 0x6203, rpm);  // Set PR0 velocity

  // --- Trigger PR0 motion ---
  writeRegister(node, 0x6002, 0x0010);  // Trigger PR0
}


// --- Helper: Issue PR0 absolute-position move to a node ---------------------
void pr0AbsoluteMove(ModbusMaster& node, long pos, const char* tag) {
  uint16_t pos_hi = (uint16_t)((pos >> 16) & 0xFFFF);
  uint16_t pos_lo = (uint16_t)(pos & 0xFFFF);

  Serial2.print("🔧 [");
  Serial2.print(tag);
  Serial2.println("] PR0 Absolute Homing Sequence");

  writeRegister(node, 0x6200, 0x0001);  // PR0 absolute
  writeRegister(node, 0x6201, pos_hi);  // pos hi
  writeRegister(node, 0x6202, pos_lo);  // pos lo

  // fixed motion params
  writeRegister(node, 0x6203, 0x0258);  // velocity = 600
  writeRegister(node, 0x6204, 0x0032);  // accel = 50
  writeRegister(node, 0x6205, 0x0032);  // decel = 50

  writeRegister(node, 0x6002, 0x0010);  // trigger PR0
}

// --- Helper: Start PR0 homing for all four steering motors ------------------
void startHomingAllPR0() {
  Serial2.println("⚙️ Re/Homing (PR0 Absolute) steering motors...");
  for (uint8_t i = 0; i < 4; i++) {
    homingDone[i] = false;  // reset state
    pr0AbsoluteMove(nodes[steering_ids[i]], HOME_ABS_POS[i], steering_tag[i]);
  }
  Serial2.println("✅ PR0 homing commands sent. Waiting for motors to reach their targets...");
}

// --- Helper: Stop all traction motion immediately ---------------------------
void stopAllTraction() {
  Serial2.println("⛔ Stopping all traction motors (RPM=0) due to E-STOP...");
  for (uint8_t i = 0; i < 4; i++) {
    setTractionVelocity(nodes[traction_ids[i]], 0);
  }
}