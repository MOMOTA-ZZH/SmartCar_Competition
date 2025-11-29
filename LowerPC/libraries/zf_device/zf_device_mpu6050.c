/*********************************************************************************************************************
* TC377 Opensourec Library 即（TC377 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC377 开源库的一部分
*
* TC377 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          zf_device_mpu6050
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.9.4
* 适用平台          TC377TP
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-11-03       pudding            first version
* 2023-04-28       pudding            增加中文注释说明
********************************************************************************************************************/
/*********************************************************************************************************************
* 接线定义：
*                  ------------------------------------
*                  模块管脚             单片机管脚
*                  软件 IIC 通信引脚对应关系
*                  SCL                查看 zf_device_mpu6050.h 中 MPU6050_SOFT_IIC_SCL 宏定义
*                  SDA                查看 zf_device_mpu6050.h 中 MPU6050_SOFT_IIC_SDA 宏定义
*                  VCC                3.3V电源
*                  GND                电源地
*                  其余引脚悬空
*
*                  硬件 IIC 通信引脚应关系
*                  SCL                查看 zf_device_mpu6050.h 中 MPU6050_IIC_SCL 宏定义
*                  SDA                查看 zf_device_mpu6050.h 中 MPU6050_IIC_SDA 宏定义
*                  VCC                3.3V电源
*                  GND                电源地
*                  其余引脚悬空
*                  ------------------------------------
********************************************************************************************************************/

#include "zf_device_mpu6050.h"

int16_t mpu6050_gyro_x = 0, mpu6050_gyro_y = 0, mpu6050_gyro_z = 0;                       // 三轴陀螺仪数据     GYRO (陀螺仪)
int16_t mpu6050_acc_x  = 0, mpu6050_acc_y  = 0, mpu6050_acc_z  = 0;                       // 三轴加速度计数据    ACC  (accelerometer 加速度计)

soft_iic_info_struct mpu6050_iic_struct;                                         // 定义 mpu6050 IIC通讯结构体
#define mpu6050_write_register(reg, data)       (soft_iic_write_8bit_register(&mpu6050_iic_struct, (reg), (data)))
#define mpu6050_read_register(reg)              (soft_iic_read_8bit_register(&mpu6050_iic_struct, (reg)))
#define mpu6050_read_registers(reg, data, len)  (soft_iic_read_8bit_registers(&mpu6050_iic_struct, (reg), (data), (len)))

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 MPU6050 加速度计数据
// 参数说明     void
// 返回参数     void
// 使用示例     mpu6050_get_acc();                              // 执行该函数后，直接查看对应的变量即可
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void mpu6050_get_acc (void)
{
    uint8 dat[6];

    mpu6050_read_registers(MPU6050_ACCEL_XOUT_H, dat, 6);
    mpu6050_acc_x = (int16_t)(((uint16)dat[0] << 8 | dat[1]));
    mpu6050_acc_y = (int16_t)(((uint16)dat[2] << 8 | dat[3]));
    mpu6050_acc_z = (int16_t)(((uint16)dat[4] << 8 | dat[5]));
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     获取 MPU6050 陀螺仪数据
// 参数说明     void
// 返回参数     void
// 使用示例     mpu6050_get_gyro();                             // 执行该函数后，直接查看对应的变量即可
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void mpu6050_get_gyro (void)
{
    uint8 dat[6];

    mpu6050_read_registers(MPU6050_GYRO_XOUT_H, dat, 6);
    mpu6050_gyro_x = (int16_t)(((uint16)dat[0] << 8 | dat[1]));
    mpu6050_gyro_y = (int16_t)(((uint16)dat[2] << 8 | dat[3]));
    mpu6050_gyro_z = (int16_t)(((uint16)dat[4] << 8 | dat[5]));
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 MPU6050
// 参数说明     void
// 返回参数     uint8           1-初始化失败 0-初始化成功
// 使用示例     mpu6050_init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
uint8 mpu6050_init (void)
{
    uint8 return_state = 0;

    soft_iic_init(&mpu6050_iic_struct, MPU6050_DEV_ADDR, MPU6050_SOFT_IIC_DELAY, MPU6050_SCL_PIN, MPU6050_SDA_PIN);

    system_delay_ms(100);                                                       // 上电延时

    do
    {
        mpu6050_write_register(MPU6050_PWR_MGMT_1, 0x00);                       // 解除休眠状态
        mpu6050_write_register(MPU6050_SMPLRT_DIV, 0);                       // 1000HZ采样率
        mpu6050_write_register(MPU6050_CONFIG, 2);

        mpu6050_write_register(MPU6050_GYRO_CONFIG, MPU6050_GYR_SAMPLE);        // 2000
        // GYRO_CONFIG寄存器
        // 设置为:0x00 陀螺仪量程为:±250 dps     获取到的陀螺仪数据除以131.2         可以转化为带物理单位的数据，单位为：°/s
        // 设置为:0x08 陀螺仪量程为:±500 dps     获取到的陀螺仪数据除以65.6          可以转化为带物理单位的数据，单位为：°/s
        // 设置为:0x10 陀螺仪量程为:±1000dps     获取到的陀螺仪数据除以32.8          可以转化为带物理单位的数据，单位为：°/s
        // 设置为:0x18 陀螺仪量程为:±2000dps     获取到的陀螺仪数据除以16.4          可以转化为带物理单位的数据，单位为：°/s

        mpu6050_write_register(MPU6050_ACCEL_CONFIG, MPU6050_ACC_SAMPLE);       // 8g
        // ACCEL_CONFIG寄存器
        // 设置为:0x00 加速度计量程为:±2g          获取到的加速度计数据 除以16384      可以转化为带物理单位的数据，单位：g(m/s^2)
        // 设置为:0x08 加速度计量程为:±4g          获取到的加速度计数据 除以8192       可以转化为带物理单位的数据，单位：g(m/s^2)
        // 设置为:0x10 加速度计量程为:±8g          获取到的加速度计数据 除以4096       可以转化为带物理单位的数据，单位：g(m/s^2)
        // 设置为:0x18 加速度计量程为:±16g         获取到的加速度计数据 除以2048       可以转化为带物理单位的数据，单位：g(m/s^2)

        mpu6050_write_register(MPU6050_INT_EN_REG, 0x00);//关闭所有中断
        mpu6050_write_register(MPU6050_USER_CONTROL, 0x00);//I2C主模式关闭
        mpu6050_write_register(MPU6050_PWR_MGMT_1, 0x01);//设置CLKSEL,PLL X轴为参考
        mpu6050_write_register(MPU6050_PWR_MGMT2_REG, 0x00);//加速度与陀螺仪都工作

    }while(0);
    return return_state;
}

//用于单独初始化6050iic接口
void soft_iic_6050_init(void){

    soft_iic_init(&mpu6050_iic_struct, MPU6050_DEV_ADDR, MPU6050_SOFT_IIC_DELAY, MPU6050_SCL_PIN, MPU6050_SDA_PIN);
    system_delay_ms(100);

}

int I2C_Write(const uint8 register_name, uint32 len, const uint8 *data){
    soft_iic_write_8bit_registers(&mpu6050_iic_struct, (register_name), (data), (len));
    return 0;
}
int I2C_Read(const uint8 register_name, uint32 len, uint8 *data){
    soft_iic_read_8bit_registers(&mpu6050_iic_struct, (register_name), (data), (len));
    return 0;
}
