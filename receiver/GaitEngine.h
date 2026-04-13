/**
 * Spider Robot - Gait Engine
 * 
 * Реализация походки Tripod (чередующиеся тройки)
 * 
 * Группа A: лапы 0, 2, 4
 * Группа B: лапы 1, 3, 5
 * 
 * Фазы:
 * 1. Группа A на земле (stance), Группа B в воздухе (swing)
 * 2. Группа B на земле (stance), Группа A в воздухе (swing)
 */

#ifndef GAIT_ENGINE_H
#define GAIT_ENGINE_H

#include "Config.h"
#include "LegKinematics.h"

class GaitEngine {
private:
  LegKinematics kinematics;
  
  // Текущая фаза походки
  bool phase;  // false = группа A в фазе swing, true = группа B в фазе swing
  
  // Прогресс фазы (0.0 - 1.0)
  float phase_progress;
  
  // Параметры шага
  float step_length;
  float step_height;
  float step_frequency;
  
  // Базовые позиции лап (домашняя позиция)
  LegPosition leg_home_positions[NUM_LEGS];
  
  // Текущие целевые позиции лап
  LegPosition leg_target_positions[NUM_LEGS];
  
  // Состояние лап (на земле или в воздухе)
  bool leg_in_air[NUM_LEGS];
  
  // Время начала текущей фазы
  unsigned long phase_start_time;
  
  // Включение/выключение походки
  bool enabled;
  
  /**
   * Расчет позиции лапы в фазе переноса (swing)
   * Используем кубическую интерполяцию для плавности
   */
  LegPosition calculateSwingPosition(float progress, LegPosition start, LegPosition end, float height) {
    LegPosition pos;
    
    // Кубическая интерполяция: 3t² - 2t³
    float t = progress;
    float interpolation = 3 * t * t - 2 * t * t * t;
    
    // Горизонтальное движение
    pos.x = start.x + (end.x - start.x) * interpolation;
    
    // Вертикальное движение (параболическая траектория)
    pos.y = start.y + (end.y - start.y) * interpolation + height * sin(PI * t);
    
    return pos;
  }
  
  /**
   * Расчет позиции лапы в опорной фазе (stance)
   * Линейное движение назад
   */
  LegPosition calculateStancePosition(float progress, LegPosition start, LegPosition end) {
    LegPosition pos;
    
    pos.x = start.x + (end.x - start.x) * progress;
    pos.y = start.y + (end.y - start.y) * progress;
    
    return pos;
  }
  
  /**
   * Обновление целевых позиций для всех лап
   */
  void updateLegTargets(float dx, float dy) {
    for (int i = 0; i < NUM_LEGS; i++) {
      if (leg_in_air[i]) {
        // Лапа в воздухе - движение к новой позиции
        float progress = phase_progress;
        
        // Начальная позиция (где была лапа)
        LegPosition start_pos = leg_home_positions[i];
        start_pos.x -= step_length / 2;
        
        // Конечная позиция (куда ставим)
        LegPosition end_pos = leg_home_positions[i];
        end_pos.x += step_length / 2;
        
        // Смещение по направлению движения
        start_pos.x += dx * step_length / 2;
        end_pos.x += dx * step_length / 2;
        start_pos.y += dy * step_length / 4;
        end_pos.y += dy * step_length / 4;
        
        leg_target_positions[i] = calculateSwingPosition(
          progress, start_pos, end_pos, step_height
        );
      } else {
        // Лапа на земле - опорная фаза
        float progress = phase_progress;
        
        LegPosition start_pos = leg_home_positions[i];
        start_pos.x += step_length / 2;
        
        LegPosition end_pos = leg_home_positions[i];
        end_pos.x -= step_length / 2;
        
        // Смещение по направлению движения
        start_pos.x += dx * step_length / 2;
        end_pos.x += dx * step_length / 2;
        start_pos.y += dy * step_length / 4;
        end_pos.y += dy * step_length / 4;
        
        leg_target_positions[i] = calculateStancePosition(progress, start_pos, end_pos);
      }
    }
  }

public:
  GaitEngine() : phase(false), phase_progress(0), enabled(false) {
    step_length = STEP_LENGTH;
    step_height = STEP_HEIGHT;
    step_frequency = STEP_FREQUENCY;
    
    phase_start_time = millis();
    
    // Инициализация домашних позиций
    for (int i = 0; i < NUM_LEGS; i++) {
      leg_home_positions[i].x = LEG_HOME_X;
      leg_home_positions[i].y = LEG_HOME_Y;
      leg_target_positions[i] = leg_home_positions[i];
      leg_in_air[i] = false;
    }
  }
  
  /**
   * Инициализация
   */
  void begin() {
    enabled = false;
    phase = false;
    phase_progress = 0;
    phase_start_time = millis();
    
    // Установка домашних позиций
    resetToHome();
  }
  
  /**
   * Сброс в домашнюю позицию
   */
  void resetToHome() {
    for (int i = 0; i < NUM_LEGS; i++) {
      leg_target_positions[i] = leg_home_positions[i];
      leg_in_air[i] = false;
    }
  }
  
  /**
   * Обновление походки (вызывать в loop)
   * @param dx Направление X (-1.0 до 1.0, назад/вперёд)
   * @param dy Направление Y (-1.0 до 1.0, вправо/влево)
   */
  void update(float dx, float dy) {
    if (!enabled) {
      resetToHome();
      return;
    }
    
    // Если нет движения, стоим в домашней позиции
    if (abs(dx) < 0.05 && abs(dy) < 0.05) {
      resetToHome();
      return;
    }
    
    // Расчет прогресса фазы
    unsigned long now = millis();
    float phase_duration = 1000.0 / step_frequency;  // Длительность фазы в мс
    phase_progress = (now - phase_start_time) / phase_duration;
    
    // Смена фазы
    if (phase_progress >= 1.0) {
      phase = !phase;
      phase_progress = 0;
      phase_start_time = now;
      
      // Обновление состояния лап
      for (int i = 0; i < 3; i++) {
        leg_in_air[TRIPOD_GROUP_A[i]] = !phase;
        leg_in_air[TRIPOD_GROUP_B[i]] = phase;
      }
    }
    
    // Обновление целевых позиций
    updateLegTargets(dx, dy);
  }
  
  /**
   * Получение целевой позиции для лапы
   */
  LegPosition getLegTarget(uint8_t leg) {
    if (leg >= NUM_LEGS) {
      return leg_home_positions[0];
    }
    return leg_target_positions[leg];
  }
  
  /**
   * Получение домашней позиции для лапы
   */
  LegPosition getLegHome(uint8_t leg) {
    if (leg >= NUM_LEGS) {
      return leg_home_positions[0];
    }
    return leg_home_positions[leg];
  }
  
  /**
   * Установка домашней позиции для лапы
   */
  void setLegHome(uint8_t leg, float x, float y) {
    if (leg >= NUM_LEGS) return;
    leg_home_positions[leg].x = x;
    leg_home_positions[leg].y = y;
  }
  
  /**
   * Получение состояния лапы (в воздухе или нет)
   */
  bool isLegInAir(uint8_t leg) {
    if (leg >= NUM_LEGS) return false;
    return leg_in_air[leg];
  }
  
  /**
   * Включение/выключение походки
   */
  void setEnabled(bool enable) {
    enabled = enable;
    if (!enable) {
      resetToHome();
    }
  }
  
  /**
   * Проверка включенности
   */
  bool isEnabled() {
    return enabled;
  }
  
  /**
   * Установка параметров шага
   */
  void setStepLength(float length) {
    step_length = constrain(length, 10, 60);
  }
  
  void setStepHeight(float height) {
    step_height = constrain(height, 5, 40);
  }
  
  void setStepFrequency(float freq) {
    step_frequency = constrain(freq, 0.5, 5.0);
  }
  
  /**
   * Получение параметров шага
   */
  float getStepLength() { return step_length; }
  float getStepHeight() { return step_height; }
  float getStepFrequency() { return step_frequency; }
  
  /**
   * Получение ссылки на кинематику
   */
  LegKinematics& getKinematics() {
    return kinematics;
  }
};

#endif // GAIT_ENGINE_H
