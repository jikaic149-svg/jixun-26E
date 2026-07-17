#ifndef TASK_APP_H
#define TASK_APP_H

#include <stdbool.h>
#include <stdint.h>

#define TASK_HEADING_LETTER_LOG_CAPACITY 14U

extern volatile uint32_t g_TASK_Heartbeat;
extern volatile uint32_t g_TASK_Fault;
extern volatile uint32_t g_TASK_Ready;
extern volatile uint32_t g_TASK_MotorLocked;
extern volatile uint32_t g_TASK_OutputEnabled;
extern volatile uint32_t g_TASK_ModeFault;
extern volatile uint32_t g_TASK_UpdateCount;
extern volatile uint32_t g_TASK_NowMs;
extern volatile uint32_t g_TASK_RouteState;
extern volatile uint32_t g_TASK_CurrentNode;
extern volatile uint32_t g_TASK_NextNode;
extern volatile uint32_t g_TASK_VisitedYMask;
extern volatile uint32_t g_TASK_DwellRemainingMs;
extern volatile uint32_t g_TASK_UserKeyPressed;
extern volatile uint32_t g_TASK_UserKeyPressCount;
extern volatile uint32_t g_TASK_HeadingLetter;
extern volatile uint32_t g_TASK_HeadingLetterMask;
extern volatile uint32_t g_TASK_HeadingLetterCount;
extern volatile uint32_t g_TASK_VisionFault;
extern volatile uint32_t g_TASK_VisionGeneration;
extern volatile uint32_t g_TASK_VisionColor;
extern volatile uint32_t g_TASK_VisionShape;
extern volatile uint32_t g_TASK_VisionClass;
extern volatile uint32_t g_TASK_VisionLocation;
extern volatile uint32_t g_TASK_VisionConfidence;
extern volatile uint8_t
    g_TASK_HeadingLetterLog[TASK_HEADING_LETTER_LOG_CAPACITY];

/* Create the statically allocated product task before starting FreeRTOS. */
bool Task_Init(void);

#endif
