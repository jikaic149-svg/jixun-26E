/**
 * @file    bluetooth.c
 * @brief   Device层 — 蓝牙通信模块实现
 * @note    发送: 组帧 + CRC16 + DMA
 *          接收: 字节流状态机解析
 */

#include "bluetooth.h"
#include "../APP/algorithm/crc16.h"
#include <string.h>

/* ======================== 初始化 ======================== */

void Bluetooth_Init(Bluetooth_Instance_t *inst, BT_DmaSend_t send_fn)
{
    memset(inst, 0, sizeof(Bluetooth_Instance_t));
    inst->dma_send = send_fn;
    inst->rx_state = BT_RX_WAIT_HEADER_0;
    inst->rx_callback = NULL;
}

void Bluetooth_RegisterRxCallback(Bluetooth_Instance_t *inst, BT_RxCmdCallback_t callback)
{
    inst->rx_callback = callback;
}

void Bluetooth_RegisterLineCallback(Bluetooth_Instance_t *inst, BT_RxLineCallback_t callback)
{
    inst->rx_line_callback = callback;
}

/* 文本行装配: 收到 \n 时去掉 \r 并补 \0, 回调上层 */
static void feed_line_byte(Bluetooth_Instance_t *inst, uint8_t b)
{
    if (b == '\n')
    {
        uint16_t len = inst->line_idx;
        while (len > 0 && inst->line_buf[len - 1] == '\r') len--;
        inst->line_buf[len] = '\0';
        if (inst->rx_line_callback)
            inst->rx_line_callback((const char *)inst->line_buf, len);
        inst->line_idx = 0;
    }
    else if (inst->line_idx < BT_TEXT_LINE_BUF_SIZE - 1)
    {
        inst->line_buf[inst->line_idx++] = b;
    }
    else
    {
        inst->line_idx = 0;  /* 溢出丢弃 */
    }
}

void Bluetooth_FeedRxByte(Bluetooth_Instance_t *inst, uint8_t b)
{
    /* 同时跑两套解析: CRC 协议状态机 + 文本行装配 */
    Bluetooth_FeedRxData(inst, &b, 1);
    feed_line_byte(inst, b);
}

/* ======================== 通用发送 ======================== */

bool Bluetooth_SendFrame(Bluetooth_Instance_t *inst, uint8_t cmd_id,
                         const uint8_t *payload, uint8_t len)
{
    if (!inst->dma_send || len > BT_MAX_PAYLOAD_SIZE)
        return false;

    uint16_t frame_len = BT_FRAME_OVERHEAD + len;

    /* 组帧 */
    inst->tx_buf[0] = BT_FRAME_HEADER_0;
    inst->tx_buf[1] = BT_FRAME_HEADER_1;
    inst->tx_buf[2] = cmd_id;
    inst->tx_buf[3] = len;

    if (len > 0 && payload != NULL)
        memcpy(&inst->tx_buf[4], payload, len);

    /* CRC16: 覆盖 Header + CMD + Len + Payload */
    uint16_t crc = crc_16(inst->tx_buf, 4 + len);
    inst->tx_buf[4 + len]     = (uint8_t)(crc >> 8);   /* CRC High */
    inst->tx_buf[4 + len + 1] = (uint8_t)(crc & 0xFF); /* CRC Low  */

    return inst->dma_send(inst->tx_buf, frame_len);
}

/* ======================== 发送姿态 ======================== */

bool Bluetooth_SendAttitude(Bluetooth_Instance_t *inst, const BT_AttitudePayload_t *att)
{
    return Bluetooth_SendFrame(inst, BT_CMD_ATTITUDE,
                               (const uint8_t *)att, sizeof(BT_AttitudePayload_t));
}

/* ======================== 接收状态机 ======================== */

void Bluetooth_FeedRxData(Bluetooth_Instance_t *inst, const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        uint8_t c = data[i];

        switch (inst->rx_state)
        {
        case BT_RX_WAIT_HEADER_0:
            if (c == BT_FRAME_HEADER_0)
                inst->rx_state = BT_RX_WAIT_HEADER_1;
            break;

        case BT_RX_WAIT_HEADER_1:
            if (c == BT_FRAME_HEADER_1)
                inst->rx_state = BT_RX_WAIT_TURN;
            else
                inst->rx_state = BT_RX_WAIT_HEADER_0;
            break;

        case BT_RX_WAIT_TURN:
            inst->rx_turn = c;
            inst->rx_state = BT_RX_WAIT_COLOR;
            break;

        case BT_RX_WAIT_COLOR:
            inst->rx_color = c;
            inst->rx_state = BT_RX_WAIT_SHAPE;
            break;

        case BT_RX_WAIT_SHAPE:
            inst->rx_shape = c;
            inst->rx_state = BT_RX_WAIT_CHECKSUM;
            break;

        case BT_RX_WAIT_CHECKSUM:
            {
                uint8_t checksum = (inst->rx_turn + inst->rx_color + inst->rx_shape) & 0xFF;
                if (c == checksum && inst->rx_callback != NULL)
                {
                    inst->rx_callback(inst->rx_turn, inst->rx_color, inst->rx_shape);
                }
                inst->rx_state = BT_RX_WAIT_HEADER_0;
            }
            break;

        default:
            inst->rx_state = BT_RX_WAIT_HEADER_0;
            break;
        }
    }
}
