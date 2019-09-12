#ifndef ROBOHAND_H
#define ROBOHAND_H

#include "stm32f4xx.h"
#include "main.h"
#include "Robohand.h"

void RobohandInit(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	
	GPIO_InitTypeDef RobohandPD8,RobohandPD9;
	
	
	RobohandPD8.GPIO_Pin = GPIO_Pin_8;
	RobohandPD8.GPIO_Mode = GPIO_Mode_OUT;
	RobohandPD8.GPIO_Speed = GPIO_Speed_50MHz;
	RobohandPD8.GPIO_OType = GPIO_OType_PP;
	RobohandPD8.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD,&RobohandPD8);
	
	RobohandPD9.GPIO_Pin = GPIO_Pin_9;
	RobohandPD9.GPIO_Mode = GPIO_Mode_OUT;
	RobohandPD9.GPIO_Speed = GPIO_Speed_50MHz;
	RobohandPD9.GPIO_OType = GPIO_OType_PP;
	RobohandPD9.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD,&RobohandPD9);
	
}





#endif