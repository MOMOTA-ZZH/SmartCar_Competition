
#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"
// ���������#pragma section all restore���֮���ȫ�ֱ���������CPU0��RAM��

// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
// �������ǿ�Դ��չ��� ��������ֲ���߲��Ը���������
     
// **************************** �������� ****************************
int core0_main(void)
{
    clock_init();                   // ��ȡʱ��Ƶ��<��ر���>
    debug_init();                   // ��ʼ��Ĭ�ϵ��Դ���
    // �˴���д�û����� ���������ʼ�������




    
    GPIO_KEY_Init();
    Directory_Init();//�˵���ʼ��
    tft180_init();
    tft180_clear();

    // �˴���д�û����� ���������ʼ�������

    cpu_wait_event_ready();         // �ȴ����к��ĳ�ʼ�����

    uart_init(UART_0,115200,UART0_TX_P14_0,UART0_RX_P14_1);//ele
    uart_rx_interrupt(UART_0, 1);// �򿪴���1�����ж�

    while (TRUE)
    {
        // �˴���д��Ҫѭ��ִ�еĴ���

        //��������
        Key_ControlM();

        TFT_Task();

        ELE_send6(Roll*100,roll_speed_ring.Ek2,encoder_bldc,pidout_3f,gyro6[0],BLDC_YAW_ANGLE_Value);//����app����

//        ANO_DT_float4(pidout_3f,pidout_2f,(float)-gyro[0],gyro6[0]);
//        ANO_DT_float4(pidout_3f,pidout_2f,Roll,(float)-gyro[0]);
        system_delay_ms(10);

        // �˴���д��Ҫѭ��ִ�еĴ���
    }
}

#pragma section all restore
// **************************** �������� ****************************

