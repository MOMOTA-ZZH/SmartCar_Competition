/*
 * circular_buffer.c
 * ���λ�����
 *  Created on: 2024��7��5��
 *      Author: 554.
 */

#include "circular_buffer.h"

/*
// ��ʼ�����λ��������洢 10 ������
circularBufferInit(&cb, sizeof(int), 10);
 */


// ��ʼ�����λ�����
void circularBufferInit(CircularBuffer *cb, size_t elementSize, size_t maxElements) {
    cb->buffer = malloc(elementSize * maxElements); // �����ڴ�
    if (cb->buffer == NULL) {
        // �����ڴ����ʧ�ܵ����
        perror("Buffer allocation failed");
        exit(EXIT_FAILURE);
    }
    cb->elementSize = elementSize; // ���õ���Ԫ�ش�С
    cb->max = maxElements;         // �����������
    cb->head = 0;                  // ��ʼ��дָ��
    cb->tail = 0;                  // ��ʼ����ָ��
    cb->full = false;              // ����Ϊδ��
}

// ���ٻ��λ�����
void circularBufferFree(CircularBuffer *cb) {
    free(cb->buffer);
}

// д�����ݵ����λ�����
bool circularBufferWrite(CircularBuffer *cb, const void *data) {
    if (cb->full) {
        return false; // ������������д��ʧ��
    }

    // ����д��λ��
    void *destination = (uint8_t *)cb->buffer + (cb->head * cb->elementSize);
    memcpy(destination, data, cb->elementSize); // �������ݵ�������

    // ����дָ��
    cb->head = (cb->head + 1) % cb->max;
    cb->full = (cb->head == cb->tail); // ������״̬

    return true;
}

// �ӻ��λ�������ȡ����
bool circularBufferRead(CircularBuffer *cb, void *data) {
    if (cb->head == cb->tail && !cb->full) {
        return false; // ������Ϊ�գ���ȡʧ��
    }

    // �����ȡλ��
    void *source = (uint8_t *)cb->buffer + (cb->tail * cb->elementSize);
    memcpy(data, source, cb->elementSize); // �������ݵ�Ŀ��

    // ���¶�ָ��
    cb->tail = (cb->tail + 1) % cb->max;
    cb->full = false; // ������״̬

    return true;
}
// ��黺�����Ƿ�Ϊ��
bool circularBufferEmpty(CircularBuffer *cb) {
    return (cb->head == cb->tail) && !cb->full;
}

// ��黺�����Ƿ�Ϊ��
bool circularBufferFull(CircularBuffer *cb) {
    return cb->full;
}

// ��ȡ��������ǰ��С
int circularBufferSize(CircularBuffer *cb) {
    if (cb->full) {
        return cb->max;
    }
    if (cb->head >= cb->tail) {
        return cb->head - cb->tail;
    } else {
        return cb->max + cb->head - cb->tail;
    }
}

bool circular_bufferAvailable(CircularBuffer *cb) {
    return cb->head != cb->tail;
}
