#include "LPC11XX.H"
unsigned int status;

#define LED1_ON()  (LPC_GPIO2->DATA &= ~(1<<0))      //����P2.0����LED
#define LED1_OFF() (LPC_GPIO2->DATA |= (1<<0))       //Ϩ��P2.0����LED
#define KEY1_DOWN() ((LPC_GPIO3->DATA & (1<<3))!=(1<<3))        //P3.3�Ű���
static volatile unsigned int ticks = 0;

/********************************************************************************
* FunctionName   : LEDInit()
* Description    : LED��ʼ��LED����
* EntryParameter : None
* ReturnValue    : None
********************************************************************************/

void LedInit(void)
{
  LPC_SYSCON->SYSAHBCLKCTRL  |=(1<<16); //ʹ��IOCONʱ��
	LPC_IOCON->PIO2_0 &= ~0x07;
	LPC_IOCON->PIO2_0 |= 0x00;//��P2.0������ΪGPIO  ����ΪGPIO���ܿ���
	LPC_SYSCON->SYSAHBCLKCTRL &= ~(1<<16);//����IOCONʱ��
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<6);
	LPC_GPIO2->DIR |=(1<<0);//��P2.0����Ϊ�������
	LPC_GPIO2->DATA |= (1<<0);    // ��P2.0����Ϊ�ߵ�ƽ
	
}
/********************************************************************************
* FunctionName   : Delayms(unit16_t ms)
* Description    : ����ϵͳ��ʱ���������뼶��ʱ
* EntryParameter : ms:Ϊ��ʱ����
* ReturnValue    : None
********************************************************************************/
void Delayms(uint16_t ms){
   SysTick-> CTRL &= ~(1<<2);  
	 SysTick->LOAD = 25000*ms-1;
	 SysTick->VAL =0;
	 SysTick->CTRL |=((1<<1)|(1<<0));
	while(! ticks);
	ticks = 0;
	SysTick->CTRL = 0;
}
/********************************************************************************

*FunctioName  :SysTickInit()
*Description  :��ʼ��ϵͳ���Ķ�ʱ��
*EntryParameter: None
*ReturnValue : None
********************************************************************************/
void SysTickInit(void)
{
   SysTick-> CTRL &= ~(1<<2); //24MHZ
	 SysTick->LOAD = 12499999;//25000*500ms-1    Ԥ��ֵ
	 SysTick->VAL =0; 
	 SysTick->CTRL |=((1<<1)|(1<<0));
	 NVIC_EnableIRQ(SysTick_IRQn); 
	
	 
}

/********************************************************************************
*FunctioName  :SysTick_Handler()
*Description  :ϵͳ���Ķ�ʱ���жϷ�����
*EntryParameter: None
*ReturnValue : None

********************************************************************************/
void SysTick_Handler(void)
{
	 ticks++;
   status = LPC_GPIO2->DATA;  //����ȥ  ��ȡ��ǰ��ֵ
	 LPC_GPIO2->DATA = ~status; //ȡ��   �ͳ���ǰ��ֵ

}

/********************************************************************************
* FunctionName   : PIOINT1_IRQHandler()
* Description    : P1���жϷ�����
* EntryParameter : None
* ReturnValue    : None
********************************************************************************/

void PIOINT3_IRQHandler()
{
    if((LPC_GPIO3->MIS & (1<<3))==(1<<3)) // P3.3������ж�
		{  
				LED1_OFF();
        if(KEY1_DOWN()){
				 LED1_ON();
				//��ʱ����
					Delayms(500);
					Delayms(500);
					Delayms(500);
					Delayms(500);
					Delayms(500);
					Delayms(500);
				}
        LPC_GPIO3->IC = (1<<3);  // ���ж�
    }
   
}


/********************************************************************************
*FunctioName  :main()
*Description  :������
*EntryParameter: None
*ReturnValue : None
********************************************************************************/

int main(void)
{
   LedInit();
	  LPC_IOCON->PIO3_3 &= ~(0x07);//P3.3����GPIO
    LPC_GPIO3->DIR    &= ~(1<<3);//P3.3����Ϊ����
    LPC_GPIO3->IE |= (1<<3); //����P3.3�����ж�
    LPC_GPIO3->IS &= ~(1<<3); // ����Ϊ���ش���
    LPC_GPIO3->IEV &= ~(1<<3); // ����Ϊ�½��ش���
	
	 SysTickInit();   
	 while(1)
	 {
		NVIC_EnableIRQ(EINT3_IRQn);
		NVIC_EnableIRQ(SysTick_IRQn);
		NVIC_SetPriority(EINT3_IRQn,1);
	 } 
}