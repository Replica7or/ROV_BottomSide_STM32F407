#include "stm32f4xx.h"                  // Device header
#include "main.h"
#include "stm32f4xx_rcc.h"
#include <stdio.h>
#include "PWM.h"
#include "Robohand.h"


char Send_buffer[256];
uint8_t Send_count= 0;

char Receive_buf[256];
uint8_t Receive_W = 0, Receive_R = 0,Receive_C = 0;

uint8_t PackToROV[19];				
uint8_t FirstBuffer[38];
uint8_t MiddleBuffer[19];
int rwrite = 0;

int X=0,Y=0,W=0,Z=0,H1,H2,H3,H4,V1,V2;

int RobohandAngle= 1500;

void LED_ini(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef myLED;
	
	myLED.GPIO_Pin = GPIO_Pin_12;
	myLED.GPIO_Mode = GPIO_Mode_OUT;
	myLED.GPIO_Speed = GPIO_Speed_2MHz;
	myLED.GPIO_OType = GPIO_OType_PP;
	myLED.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB,&myLED);
}

void PWMtest(void)
{
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	
	GPIO_InitTypeDef ThrustPA8, ThrustPA9, ThrustPA10, ThrustPA11;
	TIM_TimeBaseInitTypeDef TIM1_PWM;
	TIM_OCInitTypeDef PWM1_CH1,PWM1_CH2,PWM1_CH3,PWM1_CH4;
	
	
	
	//timer config
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
	TIM1_PWM.TIM_Prescaler = (uint16_t) ((SystemCoreClock ) / 1000000) - 1;
	TIM1_PWM.TIM_CounterMode = TIM_CounterMode_Up;
	TIM1_PWM.TIM_Period = (uint16_t) (1000000/50) - 1;
	TIM1_PWM.TIM_ClockDivision = 0;
	
	TIM_TimeBaseInit(TIM1,&TIM1_PWM);
	
	
	
	
	//CH1
	//pin PA8
	ThrustPA8.GPIO_Pin = GPIO_Pin_8;
	ThrustPA8.GPIO_Mode = GPIO_Mode_AF;
	ThrustPA8.GPIO_Speed = GPIO_Speed_50MHz;
	ThrustPA8.GPIO_OType = GPIO_OType_PP;
	ThrustPA8.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA,&ThrustPA8);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
	
	PWM1_CH1.TIM_OCMode = TIM_OCMode_PWM2;
	PWM1_CH1.TIM_OutputState = TIM_OutputState_Enable;
	PWM1_CH1.TIM_OutputNState = TIM_OutputNState_Enable;
	PWM1_CH1.TIM_OCNPolarity =TIM_OCNPolarity_High;
	PWM1_CH1.TIM_OCIdleState =TIM_OCIdleState_Set;
	PWM1_CH1.TIM_OCNIdleState=TIM_OCNIdleState_Reset;
	PWM1_CH1.TIM_Pulse = 1200;
	PWM1_CH1.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM1,&PWM1_CH1);
	
	
	//CH2
	//pin PA9
	ThrustPA9.GPIO_Pin = GPIO_Pin_9;
	ThrustPA9.GPIO_Mode = GPIO_Mode_AF;
	ThrustPA9.GPIO_Speed = GPIO_Speed_50MHz;
	ThrustPA9.GPIO_OType = GPIO_OType_PP;
	ThrustPA9.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA,&ThrustPA9);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
	
	PWM1_CH2.TIM_OCMode = TIM_OCMode_PWM2;
	PWM1_CH2.TIM_OutputState = TIM_OutputState_Enable;
	PWM1_CH2.TIM_OutputNState = TIM_OutputNState_Enable;
	PWM1_CH2.TIM_OCNPolarity =TIM_OCNPolarity_High;
	PWM1_CH2.TIM_OCIdleState =TIM_OCIdleState_Set;
	PWM1_CH2.TIM_OCNIdleState=TIM_OCNIdleState_Reset;
	PWM1_CH2.TIM_Pulse = 1200;
	PWM1_CH2.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC2Init(TIM1,&PWM1_CH2);
	
	
	//CH3
	//pin PA10
	ThrustPA10.GPIO_Pin = GPIO_Pin_10;
	ThrustPA10.GPIO_Mode = GPIO_Mode_AF;
	ThrustPA10.GPIO_Speed = GPIO_Speed_50MHz;
	ThrustPA10.GPIO_OType = GPIO_OType_PP;
	ThrustPA10.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA,&ThrustPA10);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
	
	PWM1_CH3.TIM_OCMode = TIM_OCMode_PWM2;
	PWM1_CH3.TIM_OutputState = TIM_OutputState_Enable;
	PWM1_CH3.TIM_OutputNState = TIM_OutputNState_Enable;
	PWM1_CH3.TIM_OCNPolarity =TIM_OCNPolarity_High;
	PWM1_CH3.TIM_OCIdleState =TIM_OCIdleState_Set;
	PWM1_CH3.TIM_OCNIdleState=TIM_OCNIdleState_Reset;
	PWM1_CH3.TIM_Pulse = 1200;
	PWM1_CH3.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC3Init(TIM1,&PWM1_CH3);
	
	
	//CH4
	//pin PA11
	ThrustPA11.GPIO_Pin = GPIO_Pin_11;
	ThrustPA11.GPIO_Mode = GPIO_Mode_AF;
	ThrustPA11.GPIO_Speed = GPIO_Speed_50MHz;
	ThrustPA11.GPIO_OType = GPIO_OType_PP;
	ThrustPA11.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA,&ThrustPA11);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_TIM1);
	
	PWM1_CH4.TIM_OCMode = TIM_OCMode_PWM2;
	PWM1_CH4.TIM_OutputState = TIM_OutputState_Enable;
	PWM1_CH4.TIM_OutputNState = TIM_OutputNState_Enable;
	PWM1_CH4.TIM_OCNPolarity =TIM_OCNPolarity_High;
	PWM1_CH4.TIM_OCIdleState =TIM_OCIdleState_Set;
	PWM1_CH4.TIM_OCNIdleState=TIM_OCNIdleState_Reset;
	PWM1_CH4.TIM_Pulse = 1200;
	PWM1_CH4.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC4Init(TIM1,&PWM1_CH4);
	
	TIM_CtrlPWMOutputs(TIM1,ENABLE);
	TIM_Cmd(TIM1,ENABLE);
	
	TIM_SetCompare1(TIM1,500);
	TIM_SetCompare2(TIM1,500);	
	TIM_SetCompare3(TIM1,500);
	TIM_SetCompare4(TIM1,500);
	
	
	
	
	//TIM8
	GPIO_InitTypeDef ThrustPC8,ThrustPC9,ThrustPC6;		//ThrustPC6 - manipulator
	TIM_TimeBaseInitTypeDef TIM8_PWM;
	TIM_OCInitTypeDef PWM8_CH1, PWM8_CH3,PWM8_CH4;
	
	//TIM8 Config
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
	TIM8_PWM.TIM_Prescaler = (uint16_t) ((SystemCoreClock ) / 1000000) - 1;
	TIM8_PWM.TIM_CounterMode = TIM_CounterMode_Up;
	TIM8_PWM.TIM_Period = (uint16_t) (1000000/50) - 1;
	TIM8_PWM.TIM_ClockDivision = 0;
	
	TIM_TimeBaseInit(TIM8,&TIM8_PWM);
	
	//CH1
	//pin PC6
	ThrustPC6.GPIO_Pin = GPIO_Pin_6;
	ThrustPC6.GPIO_Mode = GPIO_Mode_AF;
	ThrustPC6.GPIO_Speed = GPIO_Speed_50MHz;
	ThrustPC6.GPIO_OType = GPIO_OType_PP;
	ThrustPC6.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC,&ThrustPC6);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM8);
	
	PWM8_CH1.TIM_OCMode = TIM_OCMode_PWM2;
	PWM8_CH1.TIM_OutputState = TIM_OutputState_Enable;
	PWM8_CH1.TIM_OutputNState = TIM_OutputNState_Enable;
	PWM8_CH1.TIM_OCNPolarity =TIM_OCNPolarity_High;
	PWM8_CH1.TIM_OCIdleState =TIM_OCIdleState_Set;
	PWM8_CH1.TIM_OCNIdleState=TIM_OCNIdleState_Reset;
	PWM8_CH1.TIM_Pulse = 1200;
	PWM8_CH1.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC1Init(TIM8,&PWM8_CH1);
	
	
	//CH3
	//pin PC8
	ThrustPC8.GPIO_Pin = GPIO_Pin_8;
	ThrustPC8.GPIO_Mode = GPIO_Mode_AF;
	ThrustPC8.GPIO_Speed = GPIO_Speed_50MHz;
	ThrustPC8.GPIO_OType = GPIO_OType_PP;
	ThrustPC8.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC,&ThrustPC8);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_TIM8);
	
	PWM8_CH3.TIM_OCMode = TIM_OCMode_PWM2;
	PWM8_CH3.TIM_OutputState = TIM_OutputState_Enable;
	PWM8_CH3.TIM_OutputNState = TIM_OutputNState_Enable;
	PWM8_CH3.TIM_OCNPolarity =TIM_OCNPolarity_High;
	PWM8_CH3.TIM_OCIdleState =TIM_OCIdleState_Set;
	PWM8_CH3.TIM_OCNIdleState=TIM_OCNIdleState_Reset;
	PWM8_CH3.TIM_Pulse = 1200;
	PWM8_CH3.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC3Init(TIM8,&PWM8_CH3);
	
	//CH4
	//pin PC9
	ThrustPC9.GPIO_Pin = GPIO_Pin_9;
	ThrustPC9.GPIO_Mode = GPIO_Mode_AF;
	ThrustPC9.GPIO_Speed = GPIO_Speed_50MHz;
	ThrustPC9.GPIO_OType = GPIO_OType_PP;
	ThrustPC9.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC,&ThrustPC9);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_TIM8);
	
	PWM8_CH4.TIM_OCMode = TIM_OCMode_PWM2;
	PWM8_CH4.TIM_OutputState = TIM_OutputState_Enable;
	PWM8_CH4.TIM_OutputNState = TIM_OutputNState_Enable;
	PWM8_CH4.TIM_OCNPolarity =TIM_OCNPolarity_High;
	PWM8_CH4.TIM_OCIdleState =TIM_OCIdleState_Set;
	PWM8_CH4.TIM_OCNIdleState=TIM_OCNIdleState_Reset;
	PWM8_CH4.TIM_Pulse = 1200;
	PWM8_CH4.TIM_OCPolarity = TIM_OCPolarity_High;
	
	TIM_OC4Init(TIM8,&PWM8_CH4);
	
	TIM_CtrlPWMOutputs(TIM8,ENABLE);
	TIM_Cmd(TIM8,ENABLE);
	
	TIM_SetCompare1(TIM8,500);
	TIM_SetCompare3(TIM8,500);
	TIM_SetCompare4(TIM8,500);
}





void USART2_ini(void)
{
	//set pins 2 & 3 in AF mode for USART2
	GPIO_InitTypeDef GPIO_Init_USART;
	USART_InitTypeDef USART_InitUser;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	
	GPIO_Init_USART.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 ;
	GPIO_Init_USART.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init_USART.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init_USART.GPIO_OType = GPIO_OType_PP;
	GPIO_Init_USART.GPIO_PuPd = GPIO_PuPd_UP;
	
	GPIO_Init(GPIOA, &GPIO_Init_USART);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);
	
	//set USART2
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	
	USART_InitUser.USART_BaudRate=115200;
	USART_InitUser.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitUser.USART_Mode=USART_Mode_Tx | USART_Mode_Rx;
	USART_InitUser.USART_Parity=USART_Parity_No;
	USART_InitUser.USART_StopBits=USART_StopBits_1;
	USART_InitUser.USART_WordLength=USART_WordLength_8b;
	
	USART_Init(USART2, &USART_InitUser);
	
	NVIC_EnableIRQ(USART2_IRQn);
	
	USART_ITConfig(USART2,USART_IT_RXNE, ENABLE);
	
	USART_Cmd(USART2, ENABLE);
}




//----------------------------------------------------------
//=========================================================
void SendtoPC(void)
{
	//USART_SendData(USART2,Send_buffer[Send_count]);
			//Send_count++;
	USART_ITConfig(USART2,USART_IT_TXE, ENABLE);
}

void Thrust3()	//horizontal
{
}


void Thrust4()	//horizontal
{	
}

void Thrust6()	//horizontal
{
}

void Thrust5()	//horizontal
{
}


int Constrain(int x, int val1, int val2)
{
	if(x>val1 && x<val2)
	{
		return x;
	}
	if(x<=val1)
	{
		return val1;
	}
	if(x>=val1)
	{
		return val2;
	}
	else return x;
}


void search_head(uint8_t buf[])
{
	uint8_t index1=0,index2=0;
	for(index1=0;index1<22;index1++)
	{
		if(buf[index1]==0xAB && buf[index1+1]==0xAB)
		{
			for(index2=0;index2<11;index1++,index2++)	//find and read data frame
					MiddleBuffer[index2]=buf[index1];
			
			X=(int)MiddleBuffer[2]-107;				//change to -100 | 100
			Y=(int)MiddleBuffer[3]-107;
			W=(int)MiddleBuffer[4]-107;
			
			
			H1=Constrain(Y-X-W,-100,100)+107;	//FRONT LEFT
			H2=Constrain(Y+X+W,-100,100)+95; //FRONT RIGHT
			H3=Constrain(Y+X-W,-100,100)+107; //BACK LEFT
			H4=Constrain(Y-X+W,-100,100)+107; //BACK RIGHT
			
			V1=(int)MiddleBuffer[5];
			V2=(int)MiddleBuffer[5]-14;
			
			if(MiddleBuffer[10]==0)
			{
				TIM_SetCompare1(TIM1,H3*5+1000);
				TIM_SetCompare2(TIM1,3000-H2*5-1000);
				TIM_SetCompare3(TIM1,H1*5+1000);
				TIM_SetCompare3(TIM8,H4*5+1000);
			
				TIM_SetCompare4(TIM1,V1*5+1000);			//verical
				TIM_SetCompare4(TIM8,3000-V2*5-1000);	//vertical
			}
			else
			{
				TIM_SetCompare1(TIM1,500);
				TIM_SetCompare2(TIM1,500);
				TIM_SetCompare3(TIM1,500);
				TIM_SetCompare3(TIM8,500);
			
				TIM_SetCompare4(TIM1,500);			//verical
				TIM_SetCompare4(TIM8,500);	//vertical
			}
			
			//это было закрытие/открытие маника. Теперь это всасыватель/высасыватель провода для торпеды
			if((int)MiddleBuffer[6]==1)
			{
				//
				
				GPIO_ResetBits(GPIOD,GPIO_Pin_8);
				GPIO_SetBits(GPIOD,GPIO_Pin_9);
			}
			else 
				if((int)MiddleBuffer[6]==2)
				{
					//маник открывается
					
					GPIO_ResetBits(GPIOD,GPIO_Pin_9);
					GPIO_SetBits(GPIOD,GPIO_Pin_8);
				}
				else
				{
					//маник отключен
					GPIO_ResetBits(GPIOD,GPIO_Pin_9);
					GPIO_ResetBits(GPIOD,GPIO_Pin_8);
				}
			
			if((int)MiddleBuffer[7]==1)
			{
				RobohandAngle = RobohandAngle+75;
				if(RobohandAngle>2000) RobohandAngle=2000;
					TIM_SetCompare1(TIM11,RobohandAngle);
				//маник поворачивается влево
			}
			else 
				if((int)MiddleBuffer[7]==2)
				{
					RobohandAngle = RobohandAngle-75;
				if(RobohandAngle<1000) RobohandAngle=1000;
					TIM_SetCompare1(TIM11,RobohandAngle);
					//маник поворачивается вправо
				}
				else
				{
					TIM_SetCompare1(TIM11,RobohandAngle);
					//маник поворот отключен
				}
				
				//хуй знает что это было
			if((int)MiddleBuffer[8]==1)
			{
				GPIO_ResetBits(GPIOE,GPIO_Pin_2);
				GPIO_SetBits(GPIOE,GPIO_Pin_3);
				TIM_SetCompare1(TIM9,15000);
			}
			else 
				if((int)MiddleBuffer[8]==2)
				{
					GPIO_ResetBits(GPIOE,GPIO_Pin_3);
					GPIO_SetBits(GPIOE,GPIO_Pin_2);
					TIM_SetCompare1(TIM9,15000);
				}
				else
				{
					GPIO_ResetBits(GPIOE,GPIO_Pin_2);
					GPIO_ResetBits(GPIOE,GPIO_Pin_3);
					//TIM_SetCompare1(TIM9,0);
				}
			
				
				//теперь это движок торпеды
			if((int)MiddleBuffer[9]==1)
			{
				//GPIO_ResetBits(GPIOC,GPIO_Pin_13);
				//GPIO_SetBits(GPIOC,GPIO_Pin_14);
				TIM_SetCompare1(TIM8, 1750);
			}
			else 
				if((int)MiddleBuffer[9]==2)
				{
					//GPIO_ResetBits(GPIOC,GPIO_Pin_14);
					//GPIO_SetBits(GPIOC,GPIO_Pin_13);
					TIM_SetCompare1(TIM8, 1750);
				}
				else
				{
					//GPIO_ResetBits(GPIOE,GPIO_Pin_14);
					//GPIO_ResetBits(GPIOE,GPIO_Pin_13);
					TIM_SetCompare1(TIM8,500);
				}
				
				
				
		}
	}
}

void USART2_IRQHandler(void )
{
	/*if(USART_GetITStatus(USART2,USART_IT_TXE)==SET)
	{
		USART_ClearITPendingBit(USART2,USART_IT_TXE);
		
		if(Receive_buf[Send_count]!=0 && Send_count<3)
		{
			
			//TIM_SetCompare1(TIM1,((int)Receive_buf[Send_count])*5+1000);
			USART_SendData(USART2,Receive_buf[Send_count]);
			Send_count++;
		}
		else
		{
			Send_count = 0;
			USART_ITConfig(USART2,USART_IT_TXE, DISABLE);
		}
	}*/
	
	if(USART_GetITStatus(USART2,USART_IT_RXNE)==SET)
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
		FirstBuffer[rwrite] = USART_ReceiveData(USART2);
		if(rwrite==21)
		{
			search_head(FirstBuffer);
		}
		rwrite++;
		rwrite=rwrite%22;
		//SendtoPC();
	}
}

int main()
{
	PWMtest();
 	USART2_ini();
	ServoTIM9();
	RobohandInit();
	RobohandTurn();
	LED_ini();
	
//	GPIO_SetBits(GPIOE,GPIO_Pin_3);
//	GPIO_ResetBits(GPIOE,GPIO_Pin_2);
//	GPIO_ResetBits(GPIOC,GPIO_Pin_13);
//	GPIO_SetBits(GPIOC,GPIO_Pin_14);
	
	
	
	for(int i = 0;i<100000000;i++){}
	for(int i = 0;i<100000000;i++){}
	for(int i = 0;i<100000000;i++){}
	
	while(1){
	
	}
}
