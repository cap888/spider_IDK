/**
 * Spider Robot - Servo Controller
 * 
 * Управление 12 сервоприводами SG90 через PCA9685
 * Плавное движение без рывков
 */

#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include "Config.h"
#include <Wire.h>

// Регистры PCA9685
#define PCA9681_MODE1 0x00
#define PCA9681_MODE2 0x01
#define PCA9681_SUBADR1 0x02
#define PCA9681_SUBADR2 0x03
#define PCA9681_SUBADR3 0x04
#define PCA9681_PRESCALE 0xFE
#define PCA9681_LED0_ON_L 0x06
#define PCA9681_LED0_ON_H 0x07
#define PCA9681_LED0_OFF_L 0x08
#define PCA9681_LED0_OFF_H 0x09

class ServoController {
private:
  uint8_t i2c_address;
  
  // Текущие позиции сервоприводов (градусы)
  float current_angles[TOTAL_SERVOS];
  
  // Целевые позиции сервоприводов (градусы)
  float target_angles[TOTAL_SERVOS];
  
  // Скорости сервоприводов (градусов/с)
  float servo_speeds[TOTAL_SERVOS];
  
  // Время последнего обновления
  unsigned long last_update;
  
  /**
   * Запись регистра PCA9685
   */
  void writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(i2c_address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
  }
  
  /**
   * Чтение регистра PCA9685
   */
  uint8_t readRegister(uint8_t reg) {
    Wire.beginTransmission(i2c_address);
    Wire.write(reg);
    Wire.endTransmission();
    
    Wire.requestFrom(i2c_address, (uint8_t)1);
    return Wire.read();
  }
  
  /**
   * Установка PWM для канала
   */
  void setPWM(uint8_t channel, uint16_t on, uint16_t off) {
    Wire.beginTransmission(i2c_address);
    Wire.write(PCA9681_LED0_ON_L + 4 * channel);
    Wire.write(on & 0xFF);
    Wire.write(on >> 8);
    Wire.write(off & 0xFF);
    Wire.write(off >> 8);
    Wire.endTransmission();
  }
  
  /**
   * Преобразование угла (0-180) в значение PWM
   * SG90: 0.5ms = 0°, 2.5ms = 180° при 50Hz
   */
  uint16_t angleToPWM(float angle) {
    // Ограничение угла
    angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    
    // Расчёт PWM (примерно 102-512 для 0-180°)
    // 2.5% duty cycle = 0°, 12.5% = 180°
    // При 4096 уровнях: 102-512
    float pwm = map(angle, 0, 180, 102, 512);
    return (uint16_t)pwm;
  }

public:
  ServoController(uint8_t address = PCA9685_ADDRESS) : i2c_address(address), last_update(0) {
    // Инициализация углов
    for (int i = 0; i < TOTAL_SERVOS; i++) {
      current_angles[i] = SERVO_CENTER_ANGLE;
      target_angles[i] = SERVO_CENTER_ANGLE;
      servo_speeds[i] = SERVO_MOVE_SPEED;
    }
  }
  
  /**
   * Инициализация PCA9685
   */
  bool begin() {
    Wire.begin();
    
    // Сброс PCA9685
    writeRegister(PCA9681_MODE1, 0x00);
    delay(10);
    
    // Установка частоты PWM (50Hz для сервоприводов)
    uint8_t prescale = 25000000;  // 25 MHz
    prescale /= 4096;             // 12-bit resolution
    prescale /= SERVO_FREQUENCY;  // Desired frequency
    prescale -= 1;
    
    #ifdef DEBUG_SERVOS
    Serial.print("PCA9685 prescale: ");
    Serial.println(prescale);
    #endif
    
    // Вход в режим сна для изменения prescale
    uint8_t mode1 = readRegister(PCA9681_MODE1);
    writeRegister(PCA9681_MODE1, (mode1 & 0x7F) | 0x10);  // Set SLEEP bit
    writeRegister(PCA9681_PRESCALE, prescale);
    writeRegister(PCA9681_MODE1, mode1);  // Выход из режима сна
    delay(5);
    
    // Включение авто-инкремента
    writeRegister(PCA9681_MODE1, mode1 | 0xA0);
    
    #ifdef DEBUG_SERVOS
    Serial.println("PCA9685 initialized");
    #endif
    
    // Установка начальных позиций
    moveToInitialPosition();
    
    return true;
  }
  
  /**
   * Установка сервопривода в начальную позицию
   */
  void moveToInitialPosition() {
    for (int leg = 0; leg < NUM_LEGS; leg++) {
      setServoAngle(getBaseServoChannel(leg), INITIAL_POSITIONS[leg].base_angle);
      setServoAngle(getJointServoChannel(leg), INITIAL_POSITIONS[leg].joint_angle);
    }
    update();
  }
  
  /**
   * Установка целевого угла для сервопривода
   */
  void setServoAngle(uint8_t channel, float angle) {
    if (channel >= TOTAL_SERVOS) return;
    
    angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    target_angles[channel] = angle;
  }
  
  /**
   * Установка скорости сервопривода (градусов/с)
   */
  void setServoSpeed(uint8_t channel, float speed) {
    if (channel >= TOTAL_SERVOS) return;
    servo_speeds[channel] = speed;
  }
  
  /**
   * Установка углов для лапы
   */
  void setLegAngles(uint8_t leg, float base_angle, float joint_angle) {
    if (leg >= NUM_LEGS) return;
    
    setServoAngle(getBaseServoChannel(leg), base_angle);
    setServoAngle(getJointServoChannel(leg), joint_angle);
  }
  
  /**
   * Установка скоростей для лапы
   */
  void setLegSpeeds(uint8_t leg, float base_speed, float joint_speed) {
    if (leg >= NUM_LEGS) return;
    
    setServoSpeed(getBaseServoChannel(leg), base_speed);
    setServoSpeed(getJointServoChannel(leg), joint_speed);
  }
  
  /**
   * Обновление позиций сервоприводов (вызывать регулярно)
   * Плавное движение к целевым позициям
   */
  void update() {
    unsigned long now = millis();
    float dt = (now - last_update) / 1000.0;  // Время в секундах
    last_update = now;
    
    if (dt == 0) dt = 0.02;  // Защита от деления на ноль
    
    // Обновление каждого сервопривода
    for (int i = 0; i < TOTAL_SERVOS; i++) {
      float diff = target_angles[i] - current_angles[i];
      
      // Движение к цели с ограничением скорости
      float max_move = servo_speeds[i] * dt;
      
      if (abs(diff) > max_move) {
        current_angles[i] += (diff > 0 ? max_move : -max_move);
      } else {
        current_angles[i] = target_angles[i];
      }
      
      // Установка PWM
      uint16_t pwm = angleToPWM(current_angles[i]);
      setPWM(i, 0, pwm);
    }
  }
  
  /**
   * Принудительная установка позиции (без плавности)
   * Использовать только для калибровки!
   */
  void forceAngle(uint8_t channel, float angle) {
    if (channel >= TOTAL_SERVOS) return;
    
    angle = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    current_angles[channel] = angle;
    target_angles[channel] = angle;
    
    uint16_t pwm = angleToPWM(angle);
    setPWM(channel, 0, pwm);
  }
  
  /**
   * Получение текущего угла сервопривода
   */
  float getCurrentAngle(uint8_t channel) {
    if (channel >= TOTAL_SERVOS) return 0;
    return current_angles[channel];
  }
  
  /**
   * Получение целевого угла сервопривода
   */
  float getTargetAngle(uint8_t channel) {
    if (channel >= TOTAL_SERVOS) return 0;
    return target_angles[channel];
  }
  
  /**
   * Проверка достижения целевой позиции
   */
  bool isAtTarget(uint8_t channel, float tolerance = 1.0) {
    if (channel >= TOTAL_SERVOS) return true;
    return abs(target_angles[channel] - current_angles[channel]) <= tolerance;
  }
  
  /**
   * Проверка, все ли сервоприводы достигли цели
   */
  bool allAtTarget(float tolerance = 1.0) {
    for (int i = 0; i < TOTAL_SERVOS; i++) {
      if (!isAtTarget(i, tolerance)) return false;
    }
    return true;
  }
  
  /**
   * Получение угла базового сервопривода лапы
   */
  float getLegBaseAngle(uint8_t leg) {
    if (leg >= NUM_LEGS) return 0;
    return getCurrentAngle(getBaseServoChannel(leg));
  }
  
  /**
   * Получение угла сервопривода сустава лапы
   */
  float getLegJointAngle(uint8_t leg) {
    if (leg >= NUM_LEGS) return 0;
    return getCurrentAngle(getJointServoChannel(leg));
  }
};

#endif // SERVO_CONTROLLER_H
