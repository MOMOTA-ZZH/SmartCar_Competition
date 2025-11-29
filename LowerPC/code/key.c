/*
 * key.h
 * 按键控制
 *  Created on: 2024年9月16日
 *      Author: 554.
 */
#include "key.h"

/*************************************************************************
*  函数名称：void GPIO_KEY_Init(void)
*  功能说明：GPIO初始化函数
*  参数说明：无
*  函数返回：无
*  修改时间：2020年3月10日
*  备    注：
*************************************************************************/
void GPIO_KEY_Init(void)
{
      // 初始化,输入口，高电平
    gpio_init(KEY0p, GPI, 1, GPI_FLOATING_IN);
    gpio_init(KEY1p, GPI, 1, GPI_FLOATING_IN);
    gpio_init(KEY2p, GPI, 1, GPI_FLOATING_IN);
    gpio_init(KEY3p, GPI, 1, GPI_FLOATING_IN);


}

/*************************************************************************
*  函数名称：unsigned char KEY_Read(KEYn_e KEYno)
*  功能说明：读取按键状态
*  参数说明：KEYn_e KEYno按键编号
*  函数返回：按键状态，0/1
*  修改时间：2020年3月10日
*  备    注：
*************************************************************************/
unsigned char KEY_Read(KEYn_e KEYno)
{
    switch(KEYno)
    {
      case KEY0:
        return gpio_get_level(KEY0p);//母板上按键0
      break;

      case KEY1:
        return gpio_get_level(KEY1p);//母板上按键1
      break;

      case KEY2:
        return gpio_get_level(KEY2p);//母板上按键2
      break;

      case KEY3:
        return gpio_get_level(KEY3p);//母板上按键1
      break;

      default:
        return 0XFF;
    }
    return 0;
}
#pragma warning default     // 打开警告


/*************************************************************************
*  函数名称：unsigned char KEY_Read_All(void)
*  功能说明：读取全部按键状态
*  参数说明：无
*  函数返回：按键组合状态，0--7八种状态
*  修改时间：2020年3月10日
*  备    注：读取三个按键状态，方便组合键使用
*************************************************************************/
unsigned char KEY_Read_All(void)
{
   unsigned char tm=0;

   tm = (gpio_get_level(KEY0p)|(gpio_get_level(KEY1p)<<1)|(gpio_get_level(KEY2p)<<2));//读取各个按键状态并编码
   if(tm==0x07)
    {
       return 0;
    }
//   while(tm == (PIN_Read(KEY0p)|(PIN_Read(KEY1p)<<1)|(PIN_Read(KEY2p)<<2)));//等待按键释放

   return  (~tm)&0X07;
}

/*************************************************************************
*  函数名称：void LED_Init(void)
*  功能说明：GPIO初始化函数 LED灯所用P10.6、P10.5、P15.4和P15.6初始化
*  参数说明：无
*  函数返回：无
*  修改时间：2020年3月10日
*  备    注：
*************************************************************************/
void GPIO_LED_Init(void)
{
      // 初始化,输入口，高电平
//    gpio_init(P11_12, GPO, 1, GPO_PUSH_PULL);
//    gpio_init(P13_0, GPO, 0, GPO_PUSH_PULL);
    gpio_init(LED0p, GPO, 1, GPO_PUSH_PULL);
    gpio_init(LED1p, GPO, 1, GPO_PUSH_PULL);
}

/*************************************************************************
*  函数名称：void LED_Ctrl(LEDn_e LEDno, LEDs_e sta)
*  功能说明：LED控制
*  参数说明：LEDn_e LEDno编号,LEDs_e sta状态亮/灭/翻转
*  函数返回：按键状态，0/1
*  修改时间：2020年3月10日
*  备    注：
*************************************************************************/
void LED_Ctrl(LEDn_e LEDno, LEDs_e sta)
{
    switch(LEDno)
    {
    case LED0:
      if(sta==ON)       gpio_set_level(LED0p,0);
      else if(sta==OFF) gpio_set_level(LED0p,1);
      else if(sta==RVS) gpio_toggle_level(LED0p);
    break;

    case LED1:
      if(sta==ON)       gpio_set_level(LED1p,0);
      else if(sta==OFF) gpio_set_level(LED1p,1);
      else if(sta==RVS) gpio_toggle_level(LED1p);
    break;

    case LEDALL:
      if(sta==ON)
      {
          gpio_set_level(LED0p,0);
          gpio_set_level(LED1p,0);
      }
      else if(sta==OFF)
      {
          gpio_set_level(LED0p,1);
          gpio_set_level(LED1p,1);
      }
      else if(sta==RVS)
      {
          gpio_toggle_level(LED0p);
          gpio_toggle_level(LED1p);
      }

    break;
    default:
    break;
    }
}

