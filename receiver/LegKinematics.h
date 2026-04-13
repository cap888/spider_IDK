/**
 * Spider Robot - Leg Kinematics
 * 
 * Прямая и обратная кинематика для 2-суставной лапы
 * 
 * Кинематическая схема:
 * База → Сегмент 1 (L1) → Сустав → Сегмент 2 (L2) → Конец лапы
 * 
 * Углы:
 * - base_angle: угол поворота лапы от базы (0-180°)
 * - joint_angle: угол сгиба сустава (0-180°)
 */

#ifndef LEG_KINEMATICS_H
#define LEG_KINEMATICS_H

#include "Config.h"
#include <math.h>

class LegKinematics {
private:
  float l1;  // Длина первого сегмента
  float l2;  // Длина второго сегмента
  
  /**
   * Преобразование градусов в радианы
   */
  float degToRad(float deg) {
    return deg * PI / 180.0;
  }
  
  /**
   * Преобразование радиан в градусы
   */
  float radToDeg(float rad) {
    return rad * 180.0 / PI;
  }
  
  /**
   * Теорема косинусов:计算 угол по длинам сторон
   */
  float cosineRule(float a, float b, float c) {
    // c² = a² + b² - 2ab*cos(C)
    // cos(C) = (a² + b² - c²) / (2ab)
    float value = (a * a + b * b - c * c) / (2 * a * b);
    value = constrain(value, -1.0, 1.0);  // Защита от ошибок
    return acos(value);
  }

public:
  LegKinematics() {
    l1 = LEG_BASE_LENGTH;
    l2 = LEG_JOINT_LENGTH;
  }
  
  /**
   * Прямая кинематика
   * Углы сервоприводов → Координаты конца лапы
   * 
   * @param base_angle  Угол базового сервопривода (градусы)
   * @param joint_angle Угол сервопривода сустава (градусы)
   * @return Position {x, y} в мм
   */
  LegPosition forwardKinematics(float base_angle, float joint_angle) {
    LegPosition pos;
    
    // Преобразование углов в радианы
    float base_rad = degToRad(base_angle);
    float joint_rad = degToRad(joint_angle);
    
    // Нормализация углов (90° = вертикально вниз)
    float angle1 = base_rad - PI / 2;
    float angle2 = angle1 + joint_rad - PI;
    
    // Расчет позиции
    pos.x = l1 * cos(angle1) + l2 * cos(angle2);
    pos.y = l1 * sin(angle1) + l2 * sin(angle2);
    
    #ifdef DEBUG_KINEMATICS
    Serial.print("FK: base=");
    Serial.print(base_angle);
    Serial.print(" joint=");
    Serial.print(joint_angle);
    Serial.print(" -> x=");
    Serial.print(pos.x, 1);
    Serial.print(" y=");
    Serial.println(pos.y, 1);
    #endif
    
    return pos;
  }
  
  /**
   * Обратная кинематика
   * Координаты конца лапы → Углы сервоприводов
   * 
   * @param target_x Целевая позиция X (мм)
   * @param target_y Целевая позиция Y (мм)
   * @return Angles {base_angle, joint_angle} в градусах
   */
  LegAngles inverseKinematics(float target_x, float target_y) {
    LegAngles angles;
    
    // Расстояние от базы до цели
    float r = sqrt(target_x * target_x + target_y * target_y);
    
    // Проверка достижимости
    float max_reach = l1 + l2;
    if (r > max_reach) {
      #ifdef DEBUG_KINEMATICS
      Serial.println("IK: Target out of reach!");
      #endif
      
      // Возврат к максимально возможной позиции
      r = max_reach * 0.95;
    }
    
    if (r < 1) {
      r = 1;  // Защита от деления на ноль
    }
    
    // Угол до цели (относительно базы)
    float gamma = atan2(target_y, target_x);
    
    // Теорема косинусов для угла сустава
    float cos_theta2 = (target_x * target_x + target_y * target_y - l1 * l1 - l2 * l2) / (2 * l1 * l2);
    cos_theta2 = constrain(cos_theta2, -1.0, 1.0);
    float theta2 = acos(cos_theta2);
    
    // Угол сустава (в градусах)
    angles.joint_angle = radToDeg(theta2);
    
    // Угол первого сегмента
    float cos_theta1 = (l1 * l1 + r * r - l2 * l2) / (2 * l1 * r);
    cos_theta1 = constrain(cos_theta1, -1.0, 1.0);
    float theta1 = acos(cos_theta1);
    
    // Базовый угол (в градусах)
    float base_rad = gamma + theta1;
    angles.base_angle = radToDeg(base_rad) + 90;  // Смещение для сервопривода
    
    // Ограничение углов
    angles.base_angle = constrain(angles.base_angle, LEG_BASE_ANGLE_MIN, LEG_BASE_ANGLE_MAX);
    angles.joint_angle = constrain(angles.joint_angle, LEG_JOINT_ANGLE_MIN, LEG_JOINT_ANGLE_MAX);
    
    #ifdef DEBUG_KINEMATICS
    Serial.print("IK: x=");
    Serial.print(target_x, 1);
    Serial.print(" y=");
    Serial.print(target_y, 1);
    Serial.print(" -> base=");
    Serial.print(angles.base_angle, 1);
    Serial.print(" joint=");
    Serial.println(angles.joint_angle, 1);
    #endif
    
    return angles;
  }
  
  /**
   * Проверка достижимости позиции
   */
  bool isReachable(float x, float y) {
    float r = sqrt(x * x + y * y);
    return r <= (l1 + l2) * 0.95;  // 95% от максимальной досягаемости
  }
  
  /**
   * Расчет рабочей зоны лапы
   * Возвращает максимальный радиус
   */
  float getMaxReach() {
    return l1 + l2;
  }
  
  /**
   * Получение длин сегментов
   */
  float getL1() { return l1; }
  float getL2() { return l2; }
  
  /**
   * Установка длин сегментов (если конструкция изменилась)
   */
  void setSegmentLengths(float new_l1, float new_l2) {
    l1 = new_l1;
    l2 = new_l2;
  }
};

#endif // LEG_KINEMATICS_H
