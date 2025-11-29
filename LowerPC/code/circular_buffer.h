/*
 * circular_buffer.h
 *
 *  Created on: 2024年7月5日
 *      Author: 554.
 */

#ifndef CODE_CIRCULAR_H_
#define CODE_CIRCULAR_H_
#include "zf_common_headfile.h"

#define UART_BUFFER_SIZE 128 // 缓冲区大小

// 通用环形缓冲区定义
typedef struct {
    void *buffer;      // 缓冲区指针
    size_t elementSize; // 单个元素的大小（字节）
    size_t max;         // 缓冲区最大容量（元素个数）
    size_t head;        // 写指针
    size_t tail;        // 读指针
    bool full;          // 是否已满
} CircularBuffer;


// 初始化环形缓冲区
void circularBufferInit(CircularBuffer *cb, size_t elementSize, size_t maxElements) ;

// 销毁环形缓冲区
void circularBufferFree(CircularBuffer *cb);

// 写入数据到环形缓冲区
bool circularBufferWrite(CircularBuffer *cb, const void *data);

// 从环形缓冲区读取数据
bool circularBufferRead(CircularBuffer *cb, void *data);

// 检查缓冲区是否为空
bool circularBufferEmpty(CircularBuffer *cb);

// 检查缓冲区是否为满
bool circularBufferFull(CircularBuffer *cb);

// 获取缓冲区当前大小
int circularBufferSize(CircularBuffer *cb);

bool circular_bufferAvailable(CircularBuffer *cb);

#endif /* CODE_CIRCULAR_H_ */
