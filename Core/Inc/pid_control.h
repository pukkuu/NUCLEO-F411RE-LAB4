/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : user_function.h
  * @brief          : Header for user_function.c file.
  *                   This file contains user define function.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef PID_CONTROL
#define ID_CONTROL

#include "main.h"
#include "math.h"

// USER DEFINE VARIABLE BEGIN
#define GEAR_RATIO 64
#define PULSE_PER_REVOLUTION 12
// USER DEFINE VARIABLE END

// USER DEFINE STRUCTURE BEGIN
typedef struct {
	GPIO_TypeDef* Port;
	uint16_t Pin;
}GPIO_PortPin;

typedef struct {
	TIM_HandleTypeDef* htim;
	uint16_t TIM_CHANNEL;
	GPIO_PortPin I1;
	GPIO_PortPin I2;
}MOTOR_Structure;

typedef struct {
	float Kp;
	float Ki;
	float Kd;
	uint16_t Satuation;
	float dt;
}PID_Structure;
// USER DEFINE STRUCTURE END

// USER DEFINE PROTOTYPE FUNCTION BEGIN
void MotorSetDuty(MOTOR_Structure Mx, int32_t DutyCycle);

float MotorReadPosition(TIM_HandleTypeDef htimx);

int32_t ComputePID(PID_Structure PIDx, float ek);

float ComputeLowpassConstant(uint16_t CutoffFreq, uint16_t SamplingFreq);
// USER DEFINE PROTOTYPE FUNCTION END

#endif
