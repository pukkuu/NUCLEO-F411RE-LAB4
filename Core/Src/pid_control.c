/*
 * user_function.c
 *
 *  Created on: Feb 22, 2023
 *      Author: 08809
 */

#include <pid_control.h>

// USER DEFINE VARIABLE BEGIN

// USER DEFINE VARIABLE BEGIN

// USER DEFINE FUNCTION BEGIN
void MotorSetDuty(MOTOR_Structure Mx, int32_t DutyCycle)
{
	if (DutyCycle >= 0) {
		HAL_GPIO_WritePin(Mx.I1.Port, Mx.I1.Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(Mx.I2.Port, Mx.I2.Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(Mx.htim, Mx.TIM_CHANNEL, (uint16_t) DutyCycle);
	} else {
		HAL_GPIO_WritePin(Mx.I1.Port, Mx.I1.Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(Mx.I2.Port, Mx.I2.Pin, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(Mx.htim, Mx.TIM_CHANNEL, (uint16_t) (-1 * DutyCycle));
	}
}

float MotorReadPosition(TIM_HandleTypeDef htimx)
{
	return ((int32_t) __HAL_TIM_GET_COUNTER(&htimx)) * 360.0 / (4.0 * GEAR_RATIO * PULSE_PER_REVOLUTION);
}

int32_t ComputePID(PID_Structure PIDx, float ek)
{
	static int32_t uk = 0;
	static float e_integral = 0;
	static float ek_1 = 0;

	if (ek > 360 / (2.0 * 4.0 * GEAR_RATIO * PULSE_PER_REVOLUTION) || ek < -360 / (2.0 * 4.0 * GEAR_RATIO * PULSE_PER_REVOLUTION))
	{
		if (!((uk == PIDx.Satuation || (uk * -1) == PIDx.Satuation) && ((ek >= 0) && (e_integral >= 0) || (ek < 0) && (e_integral < 0)))) {
			e_integral += ek;
		}

		uk = (PIDx.Kp * ek) + (PIDx.Ki * e_integral * PIDx.dt) + (PIDx.Kd * (ek - ek_1) / PIDx.dt);

		if (uk > PIDx.Satuation) uk = PIDx.Satuation;
		else if (uk < -1 * PIDx.Satuation) uk = -1 * PIDx.Satuation;
	}
	else
	{
		uk = 0;
	}
	ek_1 = ek;
	return uk;
}

float ComputeLowpassConstant(uint16_t CutoffFreq, uint16_t SamplingFreq)
{
	return CutoffFreq / ((float) (CutoffFreq + SamplingFreq));
}
// USER DEFINE FUNCTION END
