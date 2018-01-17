#include "LED.h"


///构造函数
///@param gpio GPIO类对象，分配片上资源
///@param highLevelOn  是否输出高电平时为LED打开
/////////////////////////
LED::LED(GPIO &gpio,bool highLevelOn)
	:mGPIO(gpio)
{
	if(highLevelOn)
		mLevelOn=1;
	else
		mLevelOn=0;
}


////////////////////////
///翻转（开-->关 ， 关-->开）
///////////////////////
void LED::Toggle()
{
	if(mGPIO.GetLevel())
		mGPIO.SetLevel(0);
	else
		mGPIO.SetLevel(1);
}

//////////////////////
///打开LED
/////////////////////
void LED::On()
{
	mGPIO.SetLevel(mLevelOn);
}

/////////////////////
///关闭LED
////////////////////
void LED::Off()
{
	mGPIO.SetLevel(mLevelOn^1);
}

////////////////////////
///闪烁n次
///@param time 闪烁次数
///@param Interval 闪烁间隔 (ms)
///////////////////////
void LED::Blink(uint8_t time,uint16_t interval)
{
	for(uint8_t i=0;i<time;++i)
	{
		On();
		TaskManager::DelayMs(interval);
		Off();
		TaskManager::DelayMs(interval);
	}
}
////////////////////////
///两个灯一起闪烁n次
///@param 另一个灯的引用
///@param time 闪烁次数
///@param Interval 闪烁间隔(ms)
///////////////////////
void LED::Blink2(LED &led,uint8_t time,uint16_t interval)
{
	for(uint8_t i=0;i<time;++i)
	{
		On();
		led.On();
		TaskManager::DelayMs(interval);
		Off();
		led.Off();
		TaskManager::DelayMs(interval);
	}
}

////////////////////////
///两个灯交替闪烁n次
///@param 另一个灯的引用
///@param time 闪烁次数
///@param Interval 闪烁间隔(ms)
///////////////////////
void LED::Blink3(LED &led,uint8_t time,uint16_t interval)
{
	for(uint8_t i=0;i<time;++i)
	{
		On();
		led.Off();
		TaskManager::DelayMs(interval);
		Off();
		led.On();
		TaskManager::DelayMs(interval);
	}
	led.Off();
}

