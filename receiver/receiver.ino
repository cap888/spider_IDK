/**
 * Spider Robot - Receiver (Робот)
 *
 * Оборудование:
 * - Arduino UNO
 * - 12x Feetech FS90 сервоприводов (6 лап, 2 серво на лапу)
 * - Подключение напрямую через Servo.h (без PCA9685)
 *
 * Автономный режим: выполняет последовательность движений при включении
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
  
  Serial.println("Spider Robot Ready!");
}

void loop() {
  // Главный цикл управления
  robot.update();
}