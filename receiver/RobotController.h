/**
 * Spider Robot - Robot Controller
 * 
 * Главный контроллер для автономного режима:
 * - Управление походкой
 * - Кинематика и сервоприводы
 * - Автономная последовательность движений
 */

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include "Config.h"
#include "ServoController.h"
#include "GaitEngine.h"

enum RobotState {
  STATE_BLINK,       // Мигание LED во время задержки
  STATE_FORWARD,     // Шаги вперёд
  STATE_PAUSE,      // Пауза между шагами
  STATE_BACKWARD,   // Шаги назад
  STATE_LEFT_STRAFE,// Стрейф влево
  STATE_RIGHT_STRAFE,// Стрейф вправо
  STATE_ROTATE,     // Поворот 360°
  STATE_STOP        // Конец
};

class RobotController {
private:
  ServoController servos;
  GaitEngine gait;
  
  // Состояние автомата
  RobotState currentState;
  unsigned long stateStartTime;
  int stepsExecuted;
  float rotationAngle;
  
  // LED blinking
  bool ledState;
  unsigned long lastLedToggle;
  float blinkFreq;
  
  // Время последнего обновления
  unsigned long last_control_update;
  
  /**
   * Преобразование целевой позиции в углы сервоприводов и установка
   */
  void setLegPosition(uint8_t leg, float x, float y) {
    LegAngles angles = gait.getKinematics().inverseKinematics(x, y);
    servos.setLegAngles(leg, angles.base_angle, angles.joint_angle);
    servos.setLegSpeeds(leg, SERVO_MOVE_SPEED, SERVO_MOVE_SPEED);
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
  
  /**
   * Мигание LED
   */
  void updateLED(bool fast) {
    unsigned long now = millis();
    unsigned long interval = fast ? (1000 / LED_BLINK_FREQ_START) : (1000 / LED_BLINK_FREQ_RUN);
    
    if (now - lastLedToggle >= interval) {
      lastLedToggle = now;
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
    }
  }
  
  /**
   * Обработка последовательности движений
   */
  void processSequence() {
    unsigned long now = millis();
    unsigned long stateElapsed = now - stateStartTime;
    
    // Длительность одного шага (одна фаза tripod = ~0.5 сек при 2 Гц)
    unsigned long stepDuration = 500;
    unsigned long pauseDuration = PAUSE_DURATION;
    
    switch (currentState) {
      case STATE_BLINK:
        updateLED(true); // 3 Гц
        if (stateElapsed >= STARTUP_DELAY) {
          digitalWrite(LED_BUILTIN, LOW);
          currentState = STATE_FORWARD;
          stateStartTime = now;
          stepsExecuted = 0;
        }
        break;
        
      case STATE_FORWARD:
        if (stepsExecuted < STEP_COUNT) {
          if (stateElapsed >= stepDuration * (stepsExecuted + 1)) {
            stepsExecuted++;
            if (stepsExecuted >= STEP_COUNT) {
              currentState = STATE_PAUSE;
              stateStartTime = now;
              stepsExecuted = 0;
            }
          }
          gait.setEnabled(true);
          gait.update(1.0, 0.0); // вперёд
          updateLegPositions();
        }
        break;
        
      case STATE_PAUSE:
        updateLED(false); // 1 Гц
        gait.setEnabled(false);
        // Возврат в домашнюю позицию
        for (int i = 0; i < NUM_LEGS; i++) {
          LegPosition home = gait.getLegHome(i);
          setLegPosition(i, home.x, home.y);
        }
        if (stateElapsed >= pauseDuration) {
          currentState = STATE_BACKWARD;
          stateStartTime = now;
          stepsExecuted = 0;
        }
        break;
        
      case STATE_BACKWARD:
        if (stepsExecuted < STEP_COUNT) {
          if (stateElapsed >= stepDuration * (stepsExecuted + 1)) {
            stepsExecuted++;
            if (stepsExecuted >= STEP_COUNT) {
              currentState = STATE_PAUSE;
              stateStartTime = now;
              stepsExecuted = 0;
            }
          }
          gait.setEnabled(true);
          gait.update(-1.0, 0.0); // назад
          updateLegPositions();
        }
        break;
        
      case STATE_LEFT_STRAFE:
        if (stepsExecuted < STEP_COUNT) {
          if (stateElapsed >= stepDuration * (stepsExecuted + 1)) {
            stepsExecuted++;
            if (stepsExecuted >= STEP_COUNT) {
              currentState = STATE_PAUSE;
              stateStartTime = now;
              stepsExecuted = 0;
            }
          }
          gait.setEnabled(true);
          gait.update(0.0, -1.0); // влево
          updateLegPositions();
        }
        break;
        
      case STATE_RIGHT_STRAFE:
        if (stepsExecuted < STEP_COUNT) {
          if (stateElapsed >= stepDuration * (stepsExecuted + 1)) {
            stepsExecuted++;
            if (stepsExecuted >= STEP_COUNT) {
              currentState = STATE_ROTATE;
              stateStartTime = now;
              rotationAngle = 0;
            }
          }
          gait.setEnabled(true);
          gait.update(0.0, 1.0); // вправо
          updateLegPositions();
        }
        break;
        
      case STATE_ROTATE: {
        // Поворот против часовой стрелки (отрицательный rotation)
        float turnAmount = -1.0; // против часовой
        gait.setEnabled(true);
        gait.setRotation(turnAmount);
        gait.update(0.0, 0.0); // только вращение
        updateLegPositions();
        
        rotationAngle += ROTATION_SPEED * 0.02; // ~20ms interval
        if (rotationAngle >= 360.0) {
          currentState = STATE_STOP;
          gait.setEnabled(false);
          gait.setRotation(0);
          // Возврат в домашнюю позицию
          for (int i = 0; i < NUM_LEGS; i++) {
            LegPosition home = gait.getLegHome(i);
            setLegPosition(i, home.x, home.y);
          }
        }
        break;
      }
        
      case STATE_STOP:
        gait.setEnabled(false);
        digitalWrite(LED_BUILTIN, LOW);
        break;
    }
  }

public:
  RobotController() : currentState(STATE_BLINK), stateStartTime(0), 
    stepsExecuted(0), rotationAngle(0), ledState(false), lastLedToggle(0),
    blinkFreq(LED_BLINK_FREQ_START), last_control_update(0) {
  }
  
  /**
   * Инициализация всех систем
   */
  bool begin() {
    Serial.begin(115200);
    Serial.println("Spider Robot starting...");
    
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    
    // Инициализация сервоприводов
    if (!servos.begin()) {
      Serial.println("ERROR: Multiservo Shield not found!");
      return false;
    }
    Serial.println("Servo controller ready");
    
    // Инициализация походки
    gait.begin();
    Serial.println("Gait engine ready");
    
    stateStartTime = millis();
    lastLedToggle = millis();
    
    Serial.println("All systems ready!");
    Serial.println("Starting autonomous mode...");
    
    return true;
  }
  
  /**
   * Главный цикл (вызывать в loop)
   */
  void update() {
    // Проверка интервала управления
    unsigned long now = millis();
    if (now - last_control_update >= CONTROL_LOOP_INTERVAL) {
      last_control_update = now;
      
      // Обработка последовательности движений
      processSequence();
    }
    
    // Обновление сервоприводов (всегда, для плавности)
    servos.update();
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
   * Получение текущего состояния
   */
  RobotState getState() {
    return currentState;
  }
};

#endif // ROBOT_CONTROLLER_H