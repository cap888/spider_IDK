/**
 * Spider Robot - Servo Controller
 *
 * Управление 12 сервоприводами через Амперка Multiservo Shield v2
 * I2C адрес: 0x47, каналы 0-11 (лапа N: каналы N*2 и N*2+1)
 */

#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include "Config.h"
#include <Wire.h>
#include <Multiservo.h>

class ServoController {
private:
  Multiservo servos[TOTAL_SERVOS];

  float current_angles[TOTAL_SERVOS];
  float target_angles[TOTAL_SERVOS];
  float servo_speeds[TOTAL_SERVOS];

  unsigned long last_update;

public:
  ServoController() : last_update(0) {
    for (int i = 0; i < TOTAL_SERVOS; i++) {
      current_angles[i] = SERVO_CENTER_ANGLE;
      target_angles[i] = SERVO_CENTER_ANGLE;
      servo_speeds[i] = SERVO_MOVE_SPEED;
    }
  }

  bool begin() {
    Wire.begin();

    for (int i = 0; i < TOTAL_SERVOS; i++) {
      servos[i].attach(i);  // канал i на Multiservo Shield
      servos[i].write(SERVO_CENTER_ANGLE);
      delay(15);
    }

    #ifdef DEBUG_SERVOS
    Serial.println("Servos initialized");
    #endif

    moveToInitialPosition();
    return true;
  }

  void moveToInitialPosition() {
    for (int leg = 0; leg < NUM_LEGS; leg++) {
      setServoAngle(leg * 2, INITIAL_POSITIONS[leg].base_angle);
      setServoAngle(leg * 2 + 1, INITIAL_POSITIONS[leg].joint_angle);
    }
    delay(500);
  }

  void setServoAngle(uint8_t channel, float angle) {
    if (channel >= TOTAL_SERVOS) return;
    angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    target_angles[channel] = angle;
  }

  void setServoSpeed(uint8_t channel, float speed) {
    if (channel >= TOTAL_SERVOS) return;
    servo_speeds[channel] = speed;
  }

  void setLegAngles(uint8_t leg, float base_angle, float joint_angle) {
    if (leg >= NUM_LEGS) return;
    setServoAngle(leg * 2, base_angle);
    setServoAngle(leg * 2 + 1, joint_angle);
  }

  void setLegSpeeds(uint8_t leg, float base_speed, float joint_speed) {
    if (leg >= NUM_LEGS) return;
    setServoSpeed(leg * 2, base_speed);
    setServoSpeed(leg * 2 + 1, joint_speed);
  }

  void update() {
    unsigned long now = millis();
    float dt = (now - last_update) / 1000.0;
    last_update = now;

    if (dt == 0) dt = 0.02;

    for (int i = 0; i < TOTAL_SERVOS; i++) {
      float diff = target_angles[i] - current_angles[i];
      float max_move = servo_speeds[i] * dt;

      if (abs(diff) > max_move) {
        current_angles[i] += (diff > 0 ? max_move : -max_move);
      } else {
        current_angles[i] = target_angles[i];
      }

      servos[i].write(current_angles[i]);
    }
  }

  void forceAngle(uint8_t channel, float angle) {
    if (channel >= TOTAL_SERVOS) return;
    angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    current_angles[channel] = angle;
    target_angles[channel] = angle;
    servos[channel].write(angle);
  }

  float getCurrentAngle(uint8_t channel) {
    if (channel >= TOTAL_SERVOS) return 0;
    return current_angles[channel];
  }

  float getTargetAngle(uint8_t channel) {
    if (channel >= TOTAL_SERVOS) return 0;
    return target_angles[channel];
  }

  bool isAtTarget(uint8_t channel, float tolerance = 1.0) {
    if (channel >= TOTAL_SERVOS) return true;
    return abs(target_angles[channel] - current_angles[channel]) <= tolerance;
  }

  bool allAtTarget(float tolerance = 1.0) {
    for (int i = 0; i < TOTAL_SERVOS; i++) {
      if (!isAtTarget(i, tolerance)) return false;
    }
    return true;
  }

  float getLegBaseAngle(uint8_t leg) {
    if (leg >= NUM_LEGS) return 0;
    return getCurrentAngle(leg * 2);
  }

  float getLegJointAngle(uint8_t leg) {
    if (leg >= NUM_LEGS) return 0;
    return getCurrentAngle(leg * 2 + 1);
  }
};

#endif