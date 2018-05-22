#include "LPC11XX.H"
unsigned int status;

#define LED1_ON()  (LPC_GPIO2->DATA &= ~(1<<0))      //点亮P2.0引脚LED
#define LED1_OFF() (LPC_GPIO2->DATA |= (1<<0))       //熄灭P2.0引脚LED
#define KEY1_DOWN() ((LPC_GPIO3->DATA & (1<<3))!=(1<<3))        //P3.3脚按键
static volatile unsigned int ticks = 0;

/********************************************************************************
* FunctionName   : LEDInit()
* Description    : LED初始化LED引脚
* EntryParameter : None
* ReturnValue    : None
********************************************************************************/

void LedInit(void)
{
  LPC_SYSCON->SYSAHBCLKCTRL  |=(1<<16); //使能IOCON时钟
	LPC_IOCON->PIO2_0 &= ~0x07;
	LPC_IOCON->PIO2_0 |= 0x00;//把P2.0脚设置为GPIO  设置为GPIO才能控制
	LPC_SYSCON->SYSAHBCLKCTRL &= ~(1<<16);//禁能IOCON时钟
	LPC_SYSCON->SYSAHBCLKCTRL |= (1<<6);
	LPC_GPIO2->DIR |=(1<<0);//把P2.0设置为输出引脚
	LPC_GPIO2->DATA |= (1<<0);    // 把P2.0设置为高电平
	
}
/********************************************************************************
* FunctionName   : Delayms(unit16_t ms)
* Description    : 利用系统定时器产生毫秒级延时
* EntryParameter : ms:为延时参数
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
*Description  :初始化系统节拍定时器
*EntryParameter: None
*ReturnValue : None
********************************************************************************/
void SysTickInit(void)
{
   SysTick-> CTRL &= ~(1<<2); //24MHZ
	 SysTick->LOAD = 12499999;//25000*500ms-1    预置值
	 SysTick->VAL =0; 
	 SysTick->CTRL |=((1<<1)|(1<<0));
	 NVIC_EnableIRQ(SysTick_IRQn); 
	
	 
}

/********************************************************************************
*FunctioName  :SysTick_Handler()
*Description  :系统节拍定时器中断服务函数
*EntryParameter: None
*ReturnValue : None

********************************************************************************/
void SysTick_Handler(void)
{
	 ticks++;
   status = LPC_GPIO2->DATA;  //读进去  读取当前的值
	 LPC_GPIO2->DATA = ~status; //取反   送出当前的值

}

/********************************************************************************
* FunctionName   : PIOINT1_IRQHandler()
* Description    : P1口中断服务函数
* EntryParameter : None
* ReturnValue    : None
********************************************************************************/

void PIOINT3_IRQHandler()
{
    if((LPC_GPIO3->MIS & (1<<3))==(1<<3)) // P3.3引起的中断
		{  
				LED1_OFF();
        if(KEY1_DOWN()){
				 LED1_ON();
				//延时三秒
					Delayms(500);
					Delayms(500);
					Delayms(500);
					Delayms(500);
					Delayms(500);
					Delayms(500);
				}
        LPC_GPIO3->IC = (1<<3);  // 清中断
    }
   
}


/********************************************************************************
*FunctioName  :main()
*Description  :主函数
*EntryParameter: None
*ReturnValue : None
********************************************************************************/

int main(void)
{
   LedInit();
	  LPC_IOCON->PIO3_3 &= ~(0x07);//P3.3配置GPIO
    LPC_GPIO3->DIR    &= ~(1<<3);//P3.3设置为输入
    LPC_GPIO3->IE |= (1<<3); //允许P3.3引脚中断
    LPC_GPIO3->IS &= ~(1<<3); // 设置为边沿触发
    LPC_GPIO3->IEV &= ~(1<<3); // 设置为下降沿触发
	
	 SysTickInit();   
	 while(1)
	 {
		NVIC_EnableIRQ(EINT3_IRQn);
		NVIC_EnableIRQ(SysTick_IRQn);
		NVIC_SetPriority(EINT3_IRQn,1);
	 } 
}