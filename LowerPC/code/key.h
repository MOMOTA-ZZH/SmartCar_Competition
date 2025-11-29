/*
 * key.h
 *
 *  Created on: 2024年9月16日
 *      Author: 554.
 */
#ifndef _key_h_
#define _key_h_

#include "zf_common_headfile.h"

//定义的管脚要对应实际按键
#define KEY0p      P20_8  //母板上按键0
#define KEY1p      P20_9  //母板上按键1
#define KEY2p      P20_6  //母板上按键2
#define KEY3p      P20_7  //母板上按键3

#define KEY4p      P20_12  //母板上按键1
#define KEY5p      P20_13  //母板上按键2

//定义的管脚要对应实际按键
#define LED0p      P21_4   //LED0
#define LED1p      P21_5   //LED1

#define Beep       P33_10    //母板上speaker
#define Beep_ON    gpio_high(Beep);
#define Beep_OFF   gpio_low(Beep);

//定义模块号
typedef enum
{
    KEY0=0,  //母板上按键0
    KEY1=1,
    KEY2=2,
    KEY3,
    KEY4,
    START,
    DSW0,    //母板上拨码开关0
    DSW1
} KEYn_e;

typedef enum
{
    LOW=0,  //按下
    HIGH=1, //松开
    FAIL=0xff,//错误
}KEYs_e;

typedef enum //
{
    NOKEYDOWN=0,
    KEY0DOWN=0x01,  //母板上按键0
    KEY1DOWN=0x02,
    KEY2DOWN=0x04,
    KEY01DOWN=0x03,
    KEY02DOWN=0x05,
    KEY12DOWN=0x06,
    KEY012DOWN=0x07,
    KEYError=0xFF,

}KEYdown_e;

//定义模块号
typedef enum
{
    LED0=0,  //LED0
    LED1=1,  //LED1
    LEDALL,
    BeeR
} LEDn_e;

typedef enum
{
    ON=0,  //亮
    OFF=1, //灭
    RVS=2, //反转
}LEDs_e;

//初始化
void GPIO_KEY_Init (void);
unsigned char KEY_Read (KEYn_e KEYno);
unsigned char KEY_Read_All (void);

//初始化
void GPIO_LED_Init(void);
void LED_Ctrl(LEDn_e LEDno, LEDs_e sta);

#endif
