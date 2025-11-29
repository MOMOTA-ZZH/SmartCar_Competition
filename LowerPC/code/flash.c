/*
 * flash.c
 * flash操作
 *  Created on: 2024年6月19日
 *      Author: 554.
 */
#include "flash.h"

#define FLASH_SECTION_INDEX       (0)                                 // 存储数据用的扇区
#define ins_PAGE_INDEX          (11)                                // 存储数据用的页码 倒数第一个页码

void flash_test(void){

flash_buffer_clear();//清空数据缓冲区
flash_union_buffer[0].float_type=554.666;
flash_write_page_from_buffer(FLASH_SECTION_INDEX, ins_PAGE_INDEX);//写入缓冲区的数据

flash_buffer_clear();//清空数据缓冲区
flash_read_page_to_buffer(FLASH_SECTION_INDEX, ins_PAGE_INDEX);//读取数据到缓冲区

//sprintf((char*)txt, "%f  ", flash_union_buffer[0].float_type);

while (TRUE)
{}

}


// 将double拆分为两个float
void doubleToTwoFloats(double input, float* output1, float* output2) {
    union DoubleToFloat dtf;
    dtf.d = input;
    *output1 = dtf.f[0];
    *output2 = dtf.f[1];
}

// 将两个float组合为double
double twoFloatsToDouble(float input1, float input2) {
    union DoubleToFloat dtf;
    dtf.f[0] = input1;
    dtf.f[1] = input2;
    return dtf.d;
}

void ins_flash_read(void){

//    flash_buffer_clear();//清空数据缓冲区
//    flash_read_page_to_buffer(FLASH_SECTION_INDEX, ins_PAGE_INDEX);//读取数据到缓冲区
//    int f=0;
//    for (int i = 0; i < Trg_N; i++) {
//        set_x[i] = flash_union_buffer[f++].float_type;
//        set_y[i] = flash_union_buffer[f++].float_type;
//    }
//    g=flash_union_buffer[f++].int32_type;
//    ACC_number_max=flash_union_buffer[f++].int32_type;
//    for (int i = 0; i < 10; i++) {
//        BX[i]=flash_union_buffer[f++].int32_type;
//    }
}

void ins_flash_write(void){
//    flash_buffer_clear();//清空数据缓冲区
//    int f=0;
//    for (int i = 0; i < Trg_N; i++) {
//        flash_union_buffer[f++].float_type = set_x[i];
//        flash_union_buffer[f++].float_type = set_y[i];
//    }
//    flash_union_buffer[f++].int32_type=g;
//    flash_union_buffer[f++].int32_type=ACC_number_max;
//    for (int i = 0; i < 10; i++) {
//        flash_union_buffer[f++].int32_type=BX[i];
//    }
//
//    flash_write_page_from_buffer(FLASH_SECTION_INDEX, ins_PAGE_INDEX);//写入缓冲区的数据
}

//void rtk_flash_read(void){
//
//    flash_buffer_clear();//清空数据缓冲区
//    flash_read_page_to_buffer(FLASH_SECTION_INDEX, RTK_PAGE_INDEX);//读取数据到缓冲区
//    int f=0;
//    for (int i = 0; i < Trg_N; i++) {
//        Target_rtk[i].latitude = twoFloatsToDouble(flash_union_buffer[f++].float_type,flash_union_buffer[f++].float_type);
//        Target_rtk[i].longitude = twoFloatsToDouble(flash_union_buffer[f++].float_type,flash_union_buffer[f++].float_type);
//    }
//    g=flash_union_buffer[f++].int32_type;
//    rtk_setposition_max=flash_union_buffer[f++].int32_type;
//    for (int i = 0; i < 10; i++) {
//        BX[i]=flash_union_buffer[f++].int32_type;
//    }
//}
//
//void rtk_flash_write(void){
//    flash_buffer_clear();//清空数据缓冲区
//    int f=0;
//    for (int i = 0; i < Trg_N; i++) {
//        doubleToTwoFloats(Target_rtk[i].latitude,&flash_union_buffer[f++].float_type,&flash_union_buffer[f++].float_type);
//        doubleToTwoFloats(Target_rtk[i].longitude,&flash_union_buffer[f++].float_type,&flash_union_buffer[f++].float_type);
//    }
//    flash_union_buffer[f++].int32_type=g;
//    flash_union_buffer[f++].int32_type=rtk_setposition_max;
//    for (int i = 0; i < 10; i++) {
//        flash_union_buffer[f++].int32_type=BX[i];
//    }
//
//    flash_write_page_from_buffer(FLASH_SECTION_INDEX, RTK_PAGE_INDEX);//写入缓冲区的数据
//}

