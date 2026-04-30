/**
 * Spider Robot - Configuration (Конфигурация)
 *
 * Все настройки проекта в одном файле
 * Arduino UNO + 12x Feetech FS90 (без PCA9685)
 */

#ifndef CONFIG_H
#define CONFIG_H

// ==========================================
// АППАРАТНАЯ КОНФИГУРАЦИЯ
// ==========================================

// Контроллер сервоприводов: Амперка Multiservo Shield v2
// Интерфейс: I2C (SDA=A4, SCL=A5 на Arduino UNO)
// Каналы: 0-17 (используются 0-11)
// Библиотека: Multiservo
#define MULTISERVO_I2C_ADDRESS 0x47

// Назначение каналов Multiservo Shield:
// Лапа N: канал N*2 (base), канал N*2+1 (joint)
//   Лапа 0: каналы 0, 1
//   Лапа 1: каналы 2, 3
//   Лапа 2: каналы 4, 5
//   Лапа 3: каналы 6, 7
//   Лапа 4: каналы 8, 9
//   Лапа 5: каналы 10, 11

// ==========================================
// АВТОНОМНЫЙ РЕЖИМ
// ==========================================

#define STARTUP_DELAY 2000       // Задержка при включении (мс)
#define STEP_COUNT 3             // Количество шагов
#define ROTATION_SPEED 60        // Скорость поворота (градусов/с)
#define LED_BLINK_FREQ_START 3  // Частота мигания LED при включении (Гц)
#define LED_BLINK_FREQ_RUN 1    // Частота мигания LED при паузе (Гц)
#define PAUSE_DURATION 500       // Пауза между шагами (мс)

// ==========================================
// КОНФИГУРАЦИЯ СЕРВОПРИВОДОВ
// ==========================================

#define NUM_LEGS 6
#define SERVOS_PER_LEG 2
#define TOTAL_SERVOS (NUM_LEGS * SERVOS_PER_LEG)

// Углы сервоприводов Feetech FS90 (в градусах)
#define SERVO_MIN_ANGLE 0
#define SERVO_MAX_ANGLE 180
#define SERVO_CENTER_ANGLE 90

// Калибровка начальных позиций (углы в градусах)
struct ServoCalibration {
  int base_angle;
  int joint_angle;
};

const ServoCalibration INITIAL_POSITIONS[NUM_LEGS] = {
  {90, 90},  // Лапа 0
  {90, 90},  // Лапа 1
  {90, 90},  // Лапа 2
  {90, 90},  // Лапа 3
  {90, 90},  // Лапа 4
  {90, 90}   // Лапа 5
};

// ==========================================
// ГЕОМЕТРИЯ ЛАПЫ
// ==========================================

#define LEG_BASE_LENGTH 30
#define LEG_JOINT_LENGTH 50

#define LEG_BASE_ANGLE_MIN 20
#define LEG_BASE_ANGLE_MAX 160
#define LEG_JOINT_ANGLE_MIN 30
#define LEG_JOINT_ANGLE_MAX 150

#define LEG_HOME_X 40
#define LEG_HOME_Y -30

// ==========================================
// КОНФИГУРАЦИЯ ПОХОДКИ (GAIT)
// ==========================================

const uint8_t TRIPOD_GROUP_A[3] = {0, 2, 4};
const uint8_t TRIPOD_GROUP_B[3] = {1, 3, 5};

#define STEP_HEIGHT 20
#define STEP_LENGTH 30
#define STEP_FREQUENCY 2

#define LEG_LIFT_SPEED 80
#define LEG_MOVE_SPEED 100
#define SERVO_MOVE_SPEED 60

// ==========================================
// ТАЙМИНГИ
// ==========================================

#define CONTROL_LOOP_INTERVAL 20

// ==========================================
// ОТЛАДКА
// ==========================================

// #define DEBUG_SERVOS
// #define DEBUG_KINEMATICS
// #define DEBUG_GAIT

// ==========================================
// СТРУКТУРЫ ДАННЫХ
// ==========================================

struct LegPosition {
  float x;
  float y;
};

struct LegAngles {
  float base_angle;
  float joint_angle;
};

#endif