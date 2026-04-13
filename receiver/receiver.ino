/**
 * Spider Robot - Receiver (Робот)
 * 
 * Оборудование:
 * - Arduino Mega
 * - Bluetooth HC-05 (Slave mode)
 * - PCA9685 (16-channel PWM driver)
 * - 12x SG90 сервоприводов (6 лап, 2 серво на лапу)
 * 
 * Главная программа, объединяющая все компоненты
 */

#include "RobotController.h"

RobotController robot;

void setup() {
  // Инициализация всех систем
  if (!robot.begin()) {
    // Ошибка инициализации - мигание встроенным LED
    pinMode(LED_BUILTIN, OUTPUT);
    while (true) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
  }
  
  Serial.println("\n=================================");
  Serial.println("  Spider Robot Ready!");
  Serial.println("=================================");
  Serial.println("Waiting for Bluetooth connection...");
}

void loop() {
  // Главный цикл управления
  robot.update();
  
  // Отладочная информация (каждую секунду)
  static unsigned long last_debug = 0;
  if (millis() - last_debug > 1000) {
    last_debug = millis();
    
    if (robot.isBluetoothConnected()) {
      Serial.print("BT: Connected | ");
      Serial.print("Speed: ");
      Serial.print(robot.getCurrentSpeed(), 2);
      Serial.print(" | ");
      Serial.print("Gait: ");
      Serial.println(robot.isGaitEnabled() ? "ON" : "OFF");
    } else {
      Serial.println("BT: Disconnected");
    }
  }
}
