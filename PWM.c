#ifndef PWM_H
#define PWM_H

#include "stm32f4xx.h"
#include "main.h"
#include "PWM.h"

void ServoTIM9(void)
{
		
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	GPIO_InitTypeDef ServoPE2,ServoPE3,ServoPC13,ServoPC14;
	
	ServoPE2.GPIO_Pin = GPIO_Pin_2;
	ServoPE2.GPIO_Mode = GPIO_Mode_OUT;
	ServoPE2.GPIO_Speed = GPIO_Speed_50MHz;
	ServoPE2.GPIO_OType = GPIO_OType_PP;
	ServoPE2.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE,&ServoPE2);
	
	ServoPE3.GPIO_Pin = GPIO_Pin_3;
	ServoPE3.GPIO_Mode = GPIO_Mode_OUT;
	ServoPE3.GPIO_Speed = GPIO_Speed_50MHz;
	ServoPE3.GPIO_OType = GPIO_OType_PP;
	ServoPE3.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE,&ServoPE3);
	
	ServoPC13.GPIO_Pin = GPIO_Pin_13;
	ServoPC13.GPIO_Mode = GPIO_Mode_OUT;
	ServoPC13.GPIO_Speed = GPIO_Speed_50MHz;
	ServoPC13.GPIO_OType = GPIO_OType_PP;
	ServoPC13.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC,&ServoPC13);
	
	ServoPC14.GPIO_Pin = GPIO_Pin_14;
	ServoPC14.GPIO_Mode = GPIO_Mode_OUT;
	ServoPC14.GPIO_Speed = GPIO_Speed_50MHz;
	ServoPC14.GPIO_OType = GPIO_OType_PP;
	ServoPC14.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC,&ServoPC14);
	


	
//for TIM9
	GPIO_InitTypeDef ThrustPE5, ThrustPE6;
	TIM_TimeBaseInitTypeDef TIM9_PWM;
	TIM_OCInitTypeDef PWM9_CH1,PWM9_CH2;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);
	
	TIM9_PWM.TIM_Prescaler = (uint16_t) ((SystemCoreClock ) / 1000000) - 1;
	TIM9_PWM.TIM_CounterMode = TIM_CounterMode_Up;
	TIM9_PWM.TIM_Period = (uint16_t) (1000000 / 50) - 1;
	TIM9_PWM.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(TIM9,&TIM9_PWM);
		
	//CH1
	//PWM-M1
	ThrustPE5.GPIO_Pin = GPIO_Pin_5;
	ThrustPE5.GPIO_Mode = GPIO_Mode_AF;
	ThrustPE5.GPIO_Speed = GPIO_Speed_50MHz;
	ThrustPE5.GPIO_OType = GPIO_OType_PP;
	ThrustPE5.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE,&ThrustPE5);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);
	
	PWM9_CH1.TIM_OCMode = TIM_OCMode_PWM1;
	PWM9_CH1.TIM_OutputState = TIM_OutputState_Enable;
	PWM9_CH1.TIM_Pulse = 0;
	PWM9_CH1.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM9,&PWM9_CH1);
	
	//CH2
	//PWM-M2
	ThrustPE6.GPIO_Pin = GPIO_Pin_6;
	ThrustPE6.GPIO_Mode = GPIO_Mode_AF;
	ThrustPE6.GPIO_Speed = GPIO_Speed_50MHz;
	ThrustPE6.GPIO_OType = GPIO_OType_PP;
	ThrustPE6.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE,&ThrustPE6);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource6, GPIO_AF_TIM9);
	
	PWM9_CH2.TIM_OCMode = TIM_OCMode_PWM1;
	PWM9_CH2.TIM_OutputState = TIM_OutputState_Enable;
	PWM9_CH2.TIM_Pulse = 0;
	PWM9_CH2.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC2Init(TIM9,&PWM9_CH2);
	
	TIM_CtrlPWMOutputs(TIM9,ENABLE);
	TIM_Cmd(TIM9,ENABLE);
	
	TIM_SetCompare1(TIM9, 15000);
	TIM_SetCompare2(TIM9, 15000);
}




void RobohandTurn(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	//TIM11
  GPIO_InitTypeDef RobohandPB9;
	TIM_TimeBaseInitTypeDef TIM11_PWM;
	TIM_OCInitTypeDef PWM11_CH1;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11,ENABLE);
	
	TIM11_PWM.TIM_Prescaler = (uint16_t) ((SystemCoreClock ) / 1000000) - 1;
	TIM11_PWM.TIM_CounterMode = TIM_CounterMode_Up;
	TIM11_PWM.TIM_Period = (uint16_t) (1000000 / 50) - 1;
	TIM11_PWM.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(TIM11,&TIM11_PWM);
		
	//CH1
	//PWM-M1
	RobohandPB9.GPIO_Pin = GPIO_Pin_9;
	RobohandPB9.GPIO_Mode = GPIO_Mode_AF;
	RobohandPB9.GPIO_Speed = GPIO_Speed_50MHz;
	RobohandPB9.GPIO_OType = GPIO_OType_PP;
	RobohandPB9.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB,&RobohandPB9);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_TIM11);
	
	PWM11_CH1.TIM_OCMode = TIM_OCMode_PWM1;
	PWM11_CH1.TIM_OutputState = TIM_OutputState_Enable;
	PWM11_CH1.TIM_Pulse = 0;
	PWM11_CH1.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM11,&PWM11_CH1);
	
		
	TIM_CtrlPWMOutputs(TIM11,ENABLE);
	TIM_Cmd(TIM11,ENABLE);
	
	TIM_SetCompare1(TIM11,500);
}
#endif