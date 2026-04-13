/**
 * Spider Robot - Robot Controller
 * 
 * Главный контроллер, объединяющий все компоненты:
 * - Приём данных Bluetooth
 * - Обработка команд джойстика
 * - Управление походкой
 * - Кинематика и сервоприводы
 */

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "Config.h"
#include "BluetoothHandler.h"
#include "ServoController.h"
#include "GaitEngine.h"

class RobotController {
private:
  BluetoothHandler bluetooth;
  ServoController servos;
  GaitEngine gait;
  
  // Время последнего обновления
  unsigned long last_control_update;
  
  // Скорость движения (адаптивная)
  float current_speed;
  float target_speed;
  
  /**
   * Преобразование целевой позиции в углы сервоприводов и установка
   */
  void setLegPosition(uint8_t leg, float x, float y) {
    // Обратная кинематика
    LegAngles angles = gait.getKinematics().inverseKinematics(x, y);
    
    // Установка углов с плавностью
    servos.setLegAngles(leg, angles.base_angle, angles.joint_angle);
    servos.setLegSpeeds(leg, SERVO_MOVE_SPEED, SERVO_MOVE_SPEED);
  }
  
  /**
   * Обработка команд Bluetooth
   */
  void processBluetooth() {
    const ControlState& state = bluetooth.getState();
    
    if (!bluetooth.isConnected()) {
      // Нет подключения - остановка
      target_speed = 0;
      gait.setEnabled(false);
      return;
    }
    
    // Расчет целевой скорости
    target_speed = sqrt(state.speed_x * state.speed_x + state.speed_y * state.speed_y);
    
    if (target_speed < 0.1) {
      // Мёртвая зона - остановка
      gait.setEnabled(false);
    } else {
      // Движение
      gait.setEnabled(true);
      
      // Нормализация направления
      float dx = state.speed_x / target_speed;
      float dy = state.speed_y / target_speed;
      
      // Обновление походки
      gait.update(dx, dy);
    }
    
    // Обработка кнопки (например, смена режима)
    if (state.button_pressed) {
      // Можно добавить смену режима здесь
      // Например, переключение между походками
    }
  }
  
  /**
   * Обновление позиций всех лап из GaitEngine
   */
  void updateLegPositions() {
    for (int i = 0; i < NUM_LEGS; i++) {
      LegPosition target = gait.getLegTarget(i);
      setLegPosition(i, target.x, target.y);
    }
  }

public:
  RobotController() : last_control_update(0), current_speed(0), target_speed(0) {
  }
  
  /**
   * Инициализация всех систем
   */
  bool begin() {
    Serial.begin(115200);
    Serial.println("Spider Robot starting...");
    
    // Инициализация Bluetooth
    bluetooth.begin();
    Serial.println("Bluetooth ready");
    
    // Инициализация сервоприводов
    if (!servos.begin()) {
      Serial.println("ERROR: PCA9685 not found!");
      return false;
    }
    Serial.println("Servo controller ready");
    
    // Инициализация походки
    gait.begin();
    Serial.println("Gait engine ready");
    
    // Небольшая задержка для стабилизации
    delay(1000);
    
    Serial.println("All systems ready!");
    return true;
  }
  
  /**
   * Главный цикл (вызывать в loop)
   */
  void update() {
    // Обновление Bluetooth
    bluetooth.update();
    
    // Проверка интервала управления
    unsigned long now = millis();
    if (now - last_control_update >= CONTROL_LOOP_INTERVAL) {
      last_control_update = now;
      
      // Обработка команд
      processBluetooth();
      
      // Обновление позиций лап
      if (gait.isEnabled()) {
        updateLegPositions();
      } else {
        // Возврат в домашнюю позицию
        for (int i = 0; i < NUM_LEGS; i++) {
          LegPosition home = gait.getLegHome(i);
          setLegPosition(i, home.x, home.y);
        }
      }
    }
    
    // Обновление сервоприводов (всегда, для плавности)
    servos.update();
  }
  
  /**
   * Аварийная остановка
   */
  void emergencyStop() {
    gait.setEnabled(false);
    
    // Плавный возврат в домашнюю позицию
    for (int i = 0; i < NUM_LEGS; i++) {
      LegPosition home = gait.getLegHome(i);
      setLegPosition(i, home.x, home.y);
    }
    
    Serial.println("EMERGENCY STOP");
  }
  
  /**
   * Получение ссылки на Bluetooth
   */
  BluetoothHandler& getBluetooth() {
    return bluetooth;
  }
  
  /**
   * Получение ссылки на сервоприводы
   */
  ServoController& getServos() {
    return servos;
  }
  
  /**
   * Получение ссылки на походку
   */
  GaitEngine& getGait() {
    return gait;
  }
  
  /**
   * Получение состояния
   */
  bool isBluetoothConnected() {
    return bluetooth.isConnected();
  }
  
  bool isGaitEnabled() {
    return gait.isEnabled();
  }
  
  float getCurrentSpeed() {
    return current_speed;
  }
};

#endif // ROBOT_CONTROLLER_H
