/*
 * CH100.c
 *
 *  Created on: 2024年10月19日
 *      Author: 554.
 */
#include "CH100.h"

static float R4(uint8_t *p) {float r; memcpy(&r,p,4); return r;}

uint8_t CH100_to_Read[85]={0};//数据数组

float gyro6[3],gyro1[3],acc[3],Pitch6,Roll6,Yaw6;

void CH100_Anl(void){

    acc[0] = R4(CH100_to_Read+18);
    acc[1] = R4(CH100_to_Read+22);
    acc[2] = R4(CH100_to_Read+26);

    gyro1[0] = R4(CH100_to_Read+30);
    gyro1[1] = R4(CH100_to_Read+34);
    gyro1[2] = R4(CH100_to_Read+38);

    Pitch6   = R4(CH100_to_Read+54);
    Yaw6     = R4(CH100_to_Read+62);

}

uint8_t num_CH100=0,long_CH100=0;

void CH100_Read_date(uint8_t bufCH100){

    switch (num_CH100)
    {
      case 0:
      if(bufCH100 == 0x5A)
       {
        num_CH100++;
       }
      else num_CH100 = 0;

      break;

      case 1:
      if(bufCH100 == 0xA5)
       {
        num_CH100++;
       }
      else num_CH100 = 0;

      break;
      case 2:
      if(bufCH100 == 0x4C)
     {
      num_CH100++;
     }
      else num_CH100 = 0;

      break;

      case 3:
      if(bufCH100 == 0x00)
     {
      num_CH100++;
     }
      else num_CH100 = 0;

      break;

      case 6:
      if(bufCH100 == 0x91)
     {
      num_CH100++;
     }
      else num_CH100 = 0;

      break;

      case 10:
      case 11:
      case 12:
      case 13:
      if(bufCH100 == 0x00)
     {
      num_CH100++;
     }
      else num_CH100 = 0;

      break;

      case 81:

      CH100_Anl();
      num_CH100 = 0;

      break;

      default:
      if(num_CH100>17 && num_CH100<42){
          CH100_to_Read[num_CH100++] = bufCH100;
      }
      else if(num_CH100>53 && num_CH100<66){
          CH100_to_Read[num_CH100++] = bufCH100;
      }
      else
      num_CH100++;

      break;

    }

}

//void CH100_Anl(void){
//
//    acc[0] = R4(CH100_to_Read+16);
//    acc[1] = R4(CH100_to_Read+20);
//    acc[2] = R4(CH100_to_Read+24);
//
//    gyro1[0] = R4(CH100_to_Read+28);
//    gyro1[1] = R4(CH100_to_Read+32);
//    gyro1[2] = R4(CH100_to_Read+36);
//
//    Pitch6   = R4(CH100_to_Read+52);
//    Yaw6     = R4(CH100_to_Read+60);
//
//}

//void CH100_Read_date(uint8_t bufCH100){
//
//    switch (num_CH100)
//    {
//      case 0:
//      if(bufCH100 == 0x4C)
//     {
//      num_CH100++;
//     }
//      else num_CH100 = 0;
//
//      break;
//
//      case 1:
//      if(bufCH100 == 0x00)
//     {
//      num_CH100++;
//     }
//      else num_CH100 = 0;
//
//      break;
//
//      case 4:
//      if(bufCH100 == 0x91)
//     {
//      num_CH100++;
//     }
//      else num_CH100 = 0;
//
//      break;
//
//      case 8:
//      case 9:
//      case 10:
//      case 11:
//      if(bufCH100 == 0x00)
//     {
//      num_CH100++;
//     }
//      else num_CH100 = 0;
//
//      break;
//
//      case 80:
//      if(bufCH100 == 0x5A)
//     {
//      num_CH100++;
//     }
//      else num_CH100 = 0;
//
//      break;
//
//      case 81:
//      if(bufCH100 == 0xA5)
//     {
//      CH100_Anl();
//     }
//      num_CH100 = 0;
//
//      break;
//
//      default:
//      if(num_CH100>15 && num_CH100<40){
//          CH100_to_Read[num_CH100++] = bufCH100;
//      }
//      else if(num_CH100>51 && num_CH100<64){
//          CH100_to_Read[num_CH100++] = bufCH100;
//      }
//      else
//      num_CH100++;
//
//      break;
//
//    }
//
//}
