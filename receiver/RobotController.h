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
  STATE_BLINK,        // Мигание LED во время задержки
  STATE_FORWARD,      // Шаги вперёд
  STATE_PAUSE,        // Пауза между движениями
  STATE_BACKWARD,     // Шаги назад
  STATE_LEFT_STRAFE,  // Стрейф влево
  STATE_RIGHT_STRAFE, // Стрейф вправо
  STATE_ROTATE,       // Поворот 360°
  STATE_STOP          // Конец
};

class RobotController {
private:
  ServoController servos;
  GaitEngine gait;
  
  // Состояние автомата
  RobotState currentState;
  RobotState nextStateAfterPause;
  unsigned long stateStartTime;
  int stepsExecuted;
  float rotationAngle;
  unsigned long lastRotationUpdate;
  
  // LED blinking
  bool ledState;
  unsigned long lastLedToggle;
  float blinkFreq;
  
  // Время последнего обновления
  unsigned long last_control_update;

#ifdef SERVO_TEST_MODE
  int testChannel;
  int testPhase;
  unsigned long testPhaseStart;
  bool testDone;
#endif
  
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

  void enterState(RobotState state) {
    currentState = state;
    stateStartTime = millis();
    stepsExecuted = 0;
  }

  void enterPause(RobotState nextState) {
    nextStateAfterPause = nextState;
    enterState(STATE_PAUSE);
  }

#ifdef SERVO_TEST_MODE
  /**
   * Безопасный тест сервоприводов: по одному каналу, малый угол.
   * Нужен до установки полной походки, чтобы проверить питание, номера каналов и направление.
   */
  void processServoTest() {
    unsigned long now = millis();

    if (testDone) {
      digitalWrite(LED_BUILTIN, LOW);
      return;
    }

    if (testChannel >= TOTAL_SERVOS) {
      Serial.println("Servo test complete");
      testDone = true;
      return;
    }

    if (now - testPhaseStart < SERVO_TEST_HOLD_MS) {
      return;
    }

    testPhaseStart = now;
    int angle = SERVO_CENTER_ANGLE;

    switch (testPhase) {
      case 0:
        angle = SERVO_CENTER_ANGLE;
        Serial.print("Test channel ");
        Serial.print(testChannel);
        Serial.println(": center");
        break;
      case 1:
        angle = SERVO_CENTER_ANGLE + SERVO_TEST_STEP_ANGLE;
        Serial.print("Test channel ");
        Serial.print(testChannel);
        Serial.println(": plus");
        break;
      case 2:
        angle = SERVO_CENTER_ANGLE - SERVO_TEST_STEP_ANGLE;
        Serial.print("Test channel ");
        Serial.print(testChannel);
        Serial.println(": minus");
        break;
      default:
        angle = SERVO_CENTER_ANGLE;
        Serial.print("Test channel ");
        Serial.print(testChannel);
        Serial.println(": done");
        testPhase = -1;
        testChannel++;
        break;
    }

    if (testChannel < TOTAL_SERVOS) {
      servos.forceAngle(testChannel, angle);
    }

    digitalWrite(LED_BUILTIN, (testPhase % 2) ? HIGH : LOW);
    testPhase++;
  }
#endif
  
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
          enterState(STATE_FORWARD);
        }
        break;
        
      case STATE_FORWARD:
        if (stepsExecuted < STEP_COUNT) {
          if (stateElapsed >= stepDuration * (stepsExecuted + 1)) {
            stepsExecuted++;
            if (stepsExecuted >= STEP_COUNT) {
              enterPause(STATE_BACKWARD);
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
          digitalWrite(LED_BUILTIN, LOW);
          enterState(nextStateAfterPause);
        }
        break;
        
      case STATE_BACKWARD:
        if (stepsExecuted < STEP_COUNT) {
          if (stateElapsed >= stepDuration * (stepsExecuted + 1)) {
            stepsExecuted++;
            if (stepsExecuted >= STEP_COUNT) {
              enterPause(STATE_LEFT_STRAFE);
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
              enterPause(STATE_RIGHT_STRAFE);
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
              enterState(STATE_ROTATE);
              rotationAngle = 0;
              lastRotationUpdate = now;
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
        
        float dt = (now - lastRotationUpdate) / 1000.0;
        lastRotationUpdate = now;
        rotationAngle += ROTATION_SPEED * dt;
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
  RobotController() : currentState(STATE_BLINK), nextStateAfterPause(STATE_FORWARD),
    stateStartTime(0), stepsExecuted(0), rotationAngle(0), lastRotationUpdate(0),
    ledState(false), lastLedToggle(0), blinkFreq(LED_BLINK_FREQ_START),
    last_control_update(0)
#ifdef SERVO_TEST_MODE
    , testChannel(0), testPhase(0), testPhaseStart(0), testDone(false)
#endif
  {
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
    lastRotationUpdate = millis();

#ifdef SERVO_TEST_MODE
    testPhaseStart = millis();
    Serial.println("SERVO_TEST_MODE enabled: autonomous gait is disabled");
#else
    Serial.println("Starting autonomous mode...");
#endif
    
    Serial.println("All systems ready!");
    
    return true;
  }
  
  /**
   * Главный цикл (вызывать в loop)
   */
  void update() {
#ifdef SERVO_TEST_MODE
    processServoTest();
    servos.update();
    return;
#endif

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