/*
 * AnoScope.c
 * 上位机发送配置以及串口接收配置区
 *  Created on: 2024年2月18日
 *      Author: 554.
 */

#include "AnoScope.h"
uint8_t data_to_send[24]={0};//发送数组
/*
【函数名】void ANO_DT_send(short data1,..., short data8 )
【功  能】匿名上位机用户协议(匿名助手新版) v1.016实测可用  只能是匿名协议V8，其他用不了!
【参数值】short data1,..., short data8, 待发送的数据
*/
void ANO_DT_send(short data1, short data2, short data3, short data4, short data5, short data6, short data7, short data8 )
{
        uint8 cnt = 0,sc = 0,ac = 0;
        uint16_t flen = data_to_send[4] + data_to_send[5] * 256;

        /*数据填充*/
        data_to_send[cnt++]=0xAB;//帧头
        data_to_send[cnt++]=0x01;//源地址
        data_to_send[cnt++]=0xFF;//目标地址
        data_to_send[cnt++]=0xF1;//功能码
        data_to_send[cnt++]=16;//数据长度（16进制低字节在前）
        data_to_send[cnt++]=0x00;//数据长度

        data_to_send[cnt++]=BYTE0(data1);
        data_to_send[cnt++]=BYTE1(data1);

        data_to_send[cnt++]=BYTE0(data2);
        data_to_send[cnt++]=BYTE1(data2);

        data_to_send[cnt++]=BYTE0(data3);
        data_to_send[cnt++]=BYTE1(data3);

        data_to_send[cnt++]=BYTE0(data4);
        data_to_send[cnt++]=BYTE1(data4);

        data_to_send[cnt++]=BYTE0(data5);
        data_to_send[cnt++]=BYTE1(data5);

        data_to_send[cnt++]=BYTE0(data6);
        data_to_send[cnt++]=BYTE1(data6);

        data_to_send[cnt++]=BYTE0(data7);
        data_to_send[cnt++]=BYTE1(data7);

        data_to_send[cnt++]=BYTE0(data8);
        data_to_send[cnt++]=BYTE1(data8);

        for(uint16_t i=0; i < (flen+6); i++)
        {
            sc += data_to_send[i]; //从帧头开始，对每一字节进行求和，直到 DATA 区结束
            ac += sc; //每一字节的求和操作，进行一次 sumcheck 的累加
        }
        //将计算出来的校验数据写入数据帧
        data_to_send[cnt++] = sc;
        data_to_send[cnt++] = ac;

        uart_write_buffer(UART_0, data_to_send, cnt);     //可以修改不同的串口发送数据;
}

void ANO_DT_float4(float data1, float data2, float data3, float data4)
{
        uint8 cnt = 0,sc = 0,ac = 0;
        uint16_t flen = data_to_send[4] + data_to_send[5] * 256;

        /*数据填充*/
        data_to_send[cnt++]=0xAB;//帧头
        data_to_send[cnt++]=0x01;//源地址
        data_to_send[cnt++]=0xFF;//目标地址
        data_to_send[cnt++]=0xF2;//功能码
        data_to_send[cnt++]=16;//数据长度（16进制低字节在前）
        data_to_send[cnt++]=0x00;//数据长度

        data_to_send[cnt++]=BYTE0(data1);
        data_to_send[cnt++]=BYTE1(data1);
        data_to_send[cnt++]=BYTE2(data1);
        data_to_send[cnt++]=BYTE3(data1);

        data_to_send[cnt++]=BYTE0(data2);
        data_to_send[cnt++]=BYTE1(data2);
        data_to_send[cnt++]=BYTE2(data2);
        data_to_send[cnt++]=BYTE3(data2);

        data_to_send[cnt++]=BYTE0(data3);
        data_to_send[cnt++]=BYTE1(data3);
        data_to_send[cnt++]=BYTE2(data3);
        data_to_send[cnt++]=BYTE3(data3);

        data_to_send[cnt++]=BYTE0(data4);
        data_to_send[cnt++]=BYTE1(data4);
        data_to_send[cnt++]=BYTE2(data4);
        data_to_send[cnt++]=BYTE3(data4);

        for(uint16_t i=0; i < (flen+6); i++)
        {
            sc += data_to_send[i]; //从帧头开始，对每一字节进行求和，直到 DATA 区结束
            ac += sc; //每一字节的求和操作，进行一次 sumcheck 的累加
        }
        //将计算出来的校验数据写入数据帧
        data_to_send[cnt++] = sc;
        data_to_send[cnt++] = ac;

        uart_write_buffer(UART_0, data_to_send, cnt);     //可以修改不同的串口发送数据;
}

uint8_t data_to_Read[50]={0};//数据数组
type_cast_t mid={0};//转换神器
/*
【函数名】void ANO_DT_Anl(void)
【功  能】匿名下位机数据解析
*/
void ANO_DT_Anl(void){

    mid.i=COMPOUND_32BIT(&data_to_Read[9]);//2
    twoKp=mid.f;
    mid.i=COMPOUND_32BIT(&data_to_Read[13]);//2
    twoKi=mid.f;
}

/*
【函数名】void ANO_DT_Read(void)
【功  能】匿名下位机接收
*/
void ANO_DT_Read(void){
    uint8_t buf0;
    static int num0=0,long0;
    uart_query_byte(UART_0, &buf0);
    if(num0 ==0 && buf0 == 0xAB)
    {
        data_to_Read[num0++] = buf0;
    }
    else if(num0 == 1 && buf0 == 0xFE)
    {
        data_to_Read[num0++] = buf0;
    }
    else if(num0 == 2 && buf0 == 0xFF)
    {
        data_to_Read[num0++] = buf0;
    }
    else if(num0 == 3 && buf0 == 0xF1)
    {
        data_to_Read[num0++] = buf0;
    }
    else if(num0 == 4)
    {
        data_to_Read[num0++] = buf0;
        long0= buf0;
    }
    else if(num0 == 5 && buf0 == 0x00)
    {
        data_to_Read[num0++] = buf0;
    }
    else if(5 < num0 && num0< 8+long0)
    {
        data_to_Read[num0++] = buf0;
        if(num0==8+long0)
        {
            num0 = 0;
            uint8 sc = 0,ac = 0;
            uint16_t flen = data_to_Read[4] + data_to_Read[5] * 256;
            for(uint16_t i=0; i < (flen+6); i++)
            {
                sc += data_to_Read[i]; //从帧头开始，对每一字节进行求和，直到 DATA 区结束
                ac += sc; //每一字节的求和操作，进行一次 sumcheck 的累加
            }
            if(sc == data_to_Read[flen+6] && ac == data_to_Read[flen+7])
                ANO_DT_Anl();//数据解析
            else num0 = 0;
        }
    }
    else    //不满足，舍弃一帧数据
    {
        num0 = 0;
    }
}

//////////////////////////////////////////////////////////////下面蓝牙部分
void ELE_send(short data1, short data2, short data3, short data4, short data5, short data6, short data7, short data8){
    uint8 cnt = 0,sc = 0;
    /*数据填充*/
    data_to_send[cnt++]=0xA5;//帧头

    data_to_send[cnt++]=BYTE0(data1);
    data_to_send[cnt++]=BYTE1(data1);

    data_to_send[cnt++]=BYTE0(data2);
    data_to_send[cnt++]=BYTE1(data2);

    data_to_send[cnt++]=BYTE0(data3);
    data_to_send[cnt++]=BYTE1(data3);

    data_to_send[cnt++]=BYTE0(data4);
    data_to_send[cnt++]=BYTE1(data4);

    data_to_send[cnt++]=BYTE0(data5);
    data_to_send[cnt++]=BYTE1(data5);

    data_to_send[cnt++]=BYTE0(data6);
    data_to_send[cnt++]=BYTE1(data6);

    data_to_send[cnt++]=BYTE0(data7);
    data_to_send[cnt++]=BYTE1(data7);

    data_to_send[cnt++]=BYTE0(data8);
    data_to_send[cnt++]=BYTE1(data8);

    for (uint16_t i=1; i < cnt; i++) {
            sc += data_to_send[i];
        }
    //将计算出来的校验数据写入数据帧
    data_to_send[cnt++] = sc;
    data_to_send[cnt++] = 0x5A;//帧尾

    uart_write_buffer(UART_1, data_to_send, cnt);     //可以修改不同的串口发送数据;
}

void ELE_send6(short data1, short data2, short data3, short data4, short data5, short data6){
    uint8 cnt = 0,sc = 0;
    /*数据填充*/
    data_to_send[cnt++]=0xA5;//帧头

    data_to_send[cnt++]=BYTE0(data1);
    data_to_send[cnt++]=BYTE1(data1);

    data_to_send[cnt++]=BYTE0(data2);
    data_to_send[cnt++]=BYTE1(data2);

    data_to_send[cnt++]=BYTE0(data3);
    data_to_send[cnt++]=BYTE1(data3);

    data_to_send[cnt++]=BYTE0(data4);
    data_to_send[cnt++]=BYTE1(data4);

    data_to_send[cnt++]=BYTE0(data5);
    data_to_send[cnt++]=BYTE1(data5);

    data_to_send[cnt++]=BYTE0(data6);
    data_to_send[cnt++]=BYTE1(data6);

    for (uint16_t i=1; i < cnt; i++) {
            sc += data_to_send[i];
        }
    //将计算出来的校验数据写入数据帧
    data_to_send[cnt++] = sc;
    data_to_send[cnt++] = 0x5A;//帧尾

    uart_write_buffer(UART_0, data_to_send, cnt);     //可以修改不同的串口发送数据;
}



void ELE_Anl(void){

    switch (data_to_Read[2])
    {
      case 0:
          mid.i=COMPOUND_32BIT(&data_to_Read[6]);//2
          roll_anglespeed_ring.Kp=mid.f;

          mid.i=COMPOUND_32BIT(&data_to_Read[10]);//2
          roll_angle_ring.Kp=mid.f;

          mid.i=COMPOUND_32BIT(&data_to_Read[14]);//2
          roll_speed_ring.Kp=mid.f;

          break;
      case 1:
          mid.i=COMPOUND_32BIT(&data_to_Read[6]);//2
          roll_anglespeed_ring.Ki=mid.f;

          mid.i=COMPOUND_32BIT(&data_to_Read[10]);//2
          roll_angle_ring.Ki=mid.f;

          mid.i=COMPOUND_32BIT(&data_to_Read[14]);//2
          roll_speed_ring.Ki=mid.f;

          break;
      case 2:
          mid.i=COMPOUND_32BIT(&data_to_Read[6]);//2
          roll_anglespeed_ring.Kd=mid.f;

          mid.i=COMPOUND_32BIT(&data_to_Read[10]);//2
          roll_angle_ring.Kd=mid.f;

          mid.i=COMPOUND_32BIT(&data_to_Read[14]);//2
          roll_speed_ring.Kd=mid.f;

          break;
      case 3:
          mid.i=COMPOUND_32BIT(&data_to_Read[10]);//2
          bdc_speed_max_set=mid.f*0.1;
          if(bdc_speed_max_set==0){
              bdc_speed_max=0;
              cross_flag=-2;
          }

          mid.i=COMPOUND_32BIT(&data_to_Read[14]);//2
          set_yaw=mid.f;

          mid.i=COMPOUND_32BIT(&data_to_Read[6]);//2
          Zero_roll=mid.f;
          ZHONGZHI_roll=Zero_roll;

          break;
      case 4:
          mid.i=COMPOUND_32BIT(&data_to_Read[6]);//2
          image_Error_wheel.Kp=mid.f;

          mid.i=COMPOUND_32BIT(&data_to_Read[10]);//2
          image_Error_wheel.Ki=mid.f;

          mid.i=COMPOUND_32BIT(&data_to_Read[14]);//2
          image_Error_wheel.Kd=mid.f;

          break;
      case 5:
          mid.i=COMPOUND_32BIT(&data_to_Read[6]);//2
          increase_dj=mid.f;

          mid.i=COMPOUND_32BIT(&data_to_Read[10]);//2
          increase_xj=mid.f;

          mid.i=COMPOUND_32BIT(&data_to_Read[14]);//2
          ROLL_SPEED_I=mid.f;

          break;
      case 6:
          mid.i=COMPOUND_32BIT(&data_to_Read[6]);//2
          Z_Coefficient=mid.f;

          mid.i=COMPOUND_32BIT(&data_to_Read[10]);//2
          Y_Coefficient=mid.f;

          mid.i=COMPOUND_32BIT(&data_to_Read[14]);//2
          Middle=mid.f;

          break;
    }

}


uint8_t num0=0,long0=0;

void ELE_Read(uint8_t buf0){

    switch (num0)
    {
      case 0:
      if(buf0 == 0xA5)
         {
             num0=1;
         }
      else num0 = 0;

      break;
      default:

      if(num0 == 1)
          {
              num0=2;
              long0= buf0;
          }
          else if(num0< 3+long0)
          {
              data_to_Read[num0++] = buf0;

          }
          else if(num0==3+long0 && buf0 == 0x5A)
          {
             num0 = 0;
             ELE_Anl();//数据解析
          }
          else    //不满足，舍弃一帧数据
              num0 = 0;
          break;

    }

}
