/**
 * @file jy61p.c
 * @brief JY61P 专用解码器底层
 */
#include "jy61p.h"
#include "../APP/broker/broker.h"
#include <string.h>

static void JY61P_Offline_Callback(void *id)
{
    JY61P_Instance *inst = (JY61P_Instance *)id;
    inst->data_ready = false;
    // 优雅降级逻辑：可选择通知上层
}

void JY61P_Init(JY61P_Instance *inst)
{
    memset(inst, 0, sizeof(JY61P_Instance));
    inst->state = JY61P_WAIT_HEADER1;
    
    Daemon_Init_Config_s cfg = {
        .reload_count = 50,    // 500ms 超时 (假设 daemon task 为 100Hz)
        .init_count   = 200,   // 初始启动给 2 秒宽限期
        .callback     = JY61P_Offline_Callback,
        .owner_id     = inst
    };
    inst->daemon = DaemonRegister(&cfg);
}

void JY61P_TareYaw(JY61P_Instance *inst)
{
    inst->yaw_offset = inst->yaw;
}

void JY61P_FIFO_Write(JY61P_Instance *inst, uint8_t data)
{
    uint16_t next_head = (inst->fifo.head + 1) % JY61P_RX_BUFFER_SIZE;
    if (next_head != inst->fifo.tail) {
        inst->fifo.buffer[inst->fifo.head] = data;
        inst->fifo.head = next_head;
    }
}

static bool JY61P_FIFO_Read(JY61P_Instance *inst, uint8_t *data)
{
    if (inst->fifo.head == inst->fifo.tail) return false;
    *data = inst->fifo.buffer[inst->fifo.tail];
    inst->fifo.tail = (inst->fifo.tail + 1) % JY61P_RX_BUFFER_SIZE;
    return true;
}

void JY61P_Parse_Task(JY61P_Instance *inst)
{
    uint8_t byte;

    while (JY61P_FIFO_Read(inst, &byte)) {
        switch (inst->state) {
            case JY61P_WAIT_HEADER1:
                if (byte == 0x55) {
                    inst->rx_frame[0] = byte;
                    inst->state = JY61P_WAIT_HEADER2;
                }
                break;

            case JY61P_WAIT_HEADER2:
                if (byte == 0x51 || byte == 0x52 || byte == 0x53) {
                    inst->rx_frame[1] = byte;
                    inst->current_type = byte;
                    inst->frame_index = 2;
                    inst->state = JY61P_RECEIVE_DATA;
                } else if (byte == 0x55) {
                    inst->state = JY61P_WAIT_HEADER2;
                } else {
                    inst->state = JY61P_WAIT_HEADER1; 
                }
                break;

            case JY61P_RECEIVE_DATA:
                inst->rx_frame[inst->frame_index++] = byte;
                
                if (inst->frame_index >= 11) {
                    uint8_t sum = 0;
                    for (uint8_t i = 0; i < 10; i++) sum += inst->rx_frame[i];
                    
                    if (sum == inst->rx_frame[10]) {
                        
                        /* ========================================================
                         * 1. 解析角度包 (0x53)
                         * ======================================================== */
                        if (inst->current_type == 0x53) {
                            int16_t roll_raw  = (int16_t)((inst->rx_frame[3] << 8) | inst->rx_frame[2]);
                            int16_t pitch_raw = (int16_t)((inst->rx_frame[5] << 8) | inst->rx_frame[4]);
                            int16_t yaw_raw   = (int16_t)((inst->rx_frame[7] << 8) | inst->rx_frame[6]);
                            inst->roll  = ((float)roll_raw / 32768.0f) * 180.0f;
                            inst->pitch = ((float)pitch_raw / 32768.0f) * 180.0f;
                            inst->yaw   = ((float)yaw_raw / 32768.0f) * 180.0f;
                            
                            // 上电收到第一帧数据时，自动将其设为 0 点偏移量
                            if (!inst->data_ready) {
                                inst->yaw_offset = inst->yaw;
                            }
                            inst->data_ready = true;
                            
                            float final_yaw = inst->yaw - inst->yaw_offset;
                            if (final_yaw > 180.0f) final_yaw -= 360.0f;
                            if (final_yaw < -180.0f) final_yaw += 360.0f;
                            
                            Broker_Publish_Float(TOPIC_JY61P_ROLL, inst->roll);
                            Broker_Publish_Float(TOPIC_JY61P_PITCH, inst->pitch);
                            Broker_Publish_Float(TOPIC_JY61P_YAW, final_yaw);
                        }
                        
                        /* ========================================================
                         * 2. 解析加速度包 (0x51)
                         * 注意：此处不再处理航向积分与推算（遵循算法解耦要求，交由上层调度）
                         * ======================================================== */
                        else if (inst->current_type == 0x51) {
                            int16_t ax_raw = (int16_t)((inst->rx_frame[3] << 8) | inst->rx_frame[2]);
                            int16_t ay_raw = (int16_t)((inst->rx_frame[5] << 8) | inst->rx_frame[4]);
                            int16_t az_raw = (int16_t)((inst->rx_frame[7] << 8) | inst->rx_frame[6]);

                            inst->ax = ((float)ax_raw / 32768.0f) * 16.0f * 9.8f;
                            inst->ay = ((float)ay_raw / 32768.0f) * 16.0f * 9.8f;
                            inst->az = ((float)az_raw / 32768.0f) * 16.0f * 9.8f;

                            Broker_Publish_Float(TOPIC_JY61P_AX, inst->ax);
                            Broker_Publish_Float(TOPIC_JY61P_AY, inst->ay);
                            Broker_Publish_Float(TOPIC_JY61P_AZ, inst->az);
                        }

                        /* ========================================================
                         * 3. 解析角速度包 (0x52)  ±2000 °/s
                         *    Wz 为串级 PID 内环反馈，零点漂移由上层校准
                         * ======================================================== */
                        else if (inst->current_type == 0x52) {
                            int16_t wx_raw = (int16_t)((inst->rx_frame[3] << 8) | inst->rx_frame[2]);
                            int16_t wy_raw = (int16_t)((inst->rx_frame[5] << 8) | inst->rx_frame[4]);
                            int16_t wz_raw = (int16_t)((inst->rx_frame[7] << 8) | inst->rx_frame[6]);

                            inst->wx = ((float)wx_raw / 32768.0f) * 2000.0f;
                            inst->wy = ((float)wy_raw / 32768.0f) * 2000.0f;
                            inst->wz = ((float)wz_raw / 32768.0f) * 2000.0f;

                            Broker_Publish_Float(TOPIC_JY61P_WX, inst->wx);
                            Broker_Publish_Float(TOPIC_JY61P_WY, inst->wy);
                            Broker_Publish_Float(TOPIC_JY61P_WZ, inst->wz);
                        }
                        
                        // 有效包解析成功，喂狗
                        if (inst->daemon) {
                            DaemonReload(inst->daemon);
                        }
                    }
                    inst->state = JY61P_WAIT_HEADER1;
                }
                break;
        }
    }
}
