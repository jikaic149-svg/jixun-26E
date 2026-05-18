/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G351X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G351X
#define CONFIG_MSPM0G3519

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_0 */
#define PWM_0_INST                                                         TIMA0
#define PWM_0_INST_IRQHandler                                   TIMA0_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMA0_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                             32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_0_C0_PORT                                                 GPIOC
#define GPIO_PWM_0_C0_PIN                                          DL_GPIO_PIN_2
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM76)
#define GPIO_PWM_0_C0_IOMUX_FUNC                     IOMUX_PINCM76_PF_TIMA0_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOC
#define GPIO_PWM_0_C1_PIN                                          DL_GPIO_PIN_4
#define GPIO_PWM_0_C1_IOMUX                                      (IOMUX_PINCM78)
#define GPIO_PWM_0_C1_IOMUX_FUNC                     IOMUX_PINCM78_PF_TIMA0_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX




/* Defines for QEI_Left */
#define QEI_Left_INST                                                      TIMG9
#define QEI_Left_INST_IRQHandler                                TIMG9_IRQHandler
#define QEI_Left_INST_INT_IRQN                                  (TIMG9_INT_IRQn)
/* Pin configuration defines for QEI_Left PHA Pin */
#define GPIO_QEI_Left_PHA_PORT                                             GPIOB
#define GPIO_QEI_Left_PHA_PIN                                     DL_GPIO_PIN_29
#define GPIO_QEI_Left_PHA_IOMUX                                  (IOMUX_PINCM66)
#define GPIO_QEI_Left_PHA_IOMUX_FUNC                 IOMUX_PINCM66_PF_TIMG9_CCP0
/* Pin configuration defines for QEI_Left PHB Pin */
#define GPIO_QEI_Left_PHB_PORT                                             GPIOB
#define GPIO_QEI_Left_PHB_PIN                                     DL_GPIO_PIN_30
#define GPIO_QEI_Left_PHB_IOMUX                                  (IOMUX_PINCM67)
#define GPIO_QEI_Left_PHB_IOMUX_FUNC                 IOMUX_PINCM67_PF_TIMG9_CCP1

/* Defines for QEI_Right */
#define QEI_Right_INST                                                     TIMG8
#define QEI_Right_INST_IRQHandler                               TIMG8_IRQHandler
#define QEI_Right_INST_INT_IRQN                                 (TIMG8_INT_IRQn)
/* Pin configuration defines for QEI_Right PHA Pin */
#define GPIO_QEI_Right_PHA_PORT                                            GPIOB
#define GPIO_QEI_Right_PHA_PIN                                     DL_GPIO_PIN_6
#define GPIO_QEI_Right_PHA_IOMUX                                 (IOMUX_PINCM23)
#define GPIO_QEI_Right_PHA_IOMUX_FUNC                IOMUX_PINCM23_PF_TIMG8_CCP0
/* Pin configuration defines for QEI_Right PHB Pin */
#define GPIO_QEI_Right_PHB_PORT                                            GPIOB
#define GPIO_QEI_Right_PHB_PIN                                     DL_GPIO_PIN_7
#define GPIO_QEI_Right_PHB_IOMUX                                 (IOMUX_PINCM24)
#define GPIO_QEI_Right_PHB_IOMUX_FUNC                IOMUX_PINCM24_PF_TIMG8_CCP1


/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                           32000000
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_32_MHZ_115200_BAUD                                      (17)
#define UART_0_FBRD_32_MHZ_115200_BAUD                                      (23)
/* Defines for UART_1 */
#define UART_1_INST                                                        UART1
#define UART_1_INST_FREQUENCY                                           32000000
#define UART_1_INST_IRQHandler                                  UART1_IRQHandler
#define UART_1_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_UART_1_RX_PORT                                                GPIOB
#define GPIO_UART_1_TX_PORT                                                GPIOB
#define GPIO_UART_1_RX_PIN                                         DL_GPIO_PIN_5
#define GPIO_UART_1_TX_PIN                                         DL_GPIO_PIN_4
#define GPIO_UART_1_IOMUX_RX                                     (IOMUX_PINCM18)
#define GPIO_UART_1_IOMUX_TX                                     (IOMUX_PINCM17)
#define GPIO_UART_1_IOMUX_RX_FUNC                      IOMUX_PINCM18_PF_UART1_RX
#define GPIO_UART_1_IOMUX_TX_FUNC                      IOMUX_PINCM17_PF_UART1_TX
#define UART_1_BAUD_RATE                                                (115200)
#define UART_1_IBRD_32_MHZ_115200_BAUD                                      (17)
#define UART_1_FBRD_32_MHZ_115200_BAUD                                      (23)
/* Defines for UART_2 */
#define UART_2_INST                                                        UART4
#define UART_2_INST_FREQUENCY                                           32000000
#define UART_2_INST_IRQHandler                                  UART4_IRQHandler
#define UART_2_INST_INT_IRQN                                      UART4_INT_IRQn
#define GPIO_UART_2_RX_PORT                                                GPIOB
#define GPIO_UART_2_TX_PORT                                                GPIOB
#define GPIO_UART_2_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_2_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_2_IOMUX_RX                                     (IOMUX_PINCM28)
#define GPIO_UART_2_IOMUX_TX                                     (IOMUX_PINCM27)
#define GPIO_UART_2_IOMUX_RX_FUNC                      IOMUX_PINCM28_PF_UART4_RX
#define GPIO_UART_2_IOMUX_TX_FUNC                      IOMUX_PINCM27_PF_UART4_TX
#define UART_2_BAUD_RATE                                                (115200)
#define UART_2_IBRD_32_MHZ_115200_BAUD                                      (17)
#define UART_2_FBRD_32_MHZ_115200_BAUD                                      (23)





/* Defines for DMA_CH0 */
#define DMA_CH0_CHAN_ID                                                      (1)
#define UART_0_INST_DMA_TRIGGER_0                            (DMA_UART0_RX_TRIG)
/* Defines for DMA_CH1 */
#define DMA_CH1_CHAN_ID                                                      (0)
#define UART_0_INST_DMA_TRIGGER_1                            (DMA_UART0_TX_TRIG)
/* Defines for DMA_CH2 */
#define DMA_CH2_CHAN_ID                                                      (5)
#define UART_1_INST_DMA_TRIGGER_0                            (DMA_UART1_RX_TRIG)
/* Defines for DMA_CH3 */
#define DMA_CH3_CHAN_ID                                                      (4)
#define UART_1_INST_DMA_TRIGGER_1                            (DMA_UART1_TX_TRIG)
/* Defines for DMA_CH4 */
#define DMA_CH4_CHAN_ID                                                      (3)
#define UART_2_INST_DMA_TRIGGER_0                            (DMA_UART4_RX_TRIG)
/* Defines for DMA_CH5 */
#define DMA_CH5_CHAN_ID                                                      (2)
#define UART_2_INST_DMA_TRIGGER_1                            (DMA_UART4_TX_TRIG)


/* Port definition for Pin Group GPIO_SWITCHES */
#define GPIO_SWITCHES_PORT                                               (GPIOB)

/* Defines for USER_SWITCH_1: GPIOB.31 with pinCMx 68 on package pin 32 */
#define GPIO_SWITCHES_USER_SWITCH_1_PIN                         (DL_GPIO_PIN_31)
#define GPIO_SWITCHES_USER_SWITCH_1_IOMUX                        (IOMUX_PINCM68)
/* Port definition for Pin Group AIN1 */
#define AIN1_PORT                                                        (GPIOA)

/* Defines for PIN_0: GPIOA.24 with pinCMx 54 on package pin 93 */
#define AIN1_PIN_0_PIN                                          (DL_GPIO_PIN_24)
#define AIN1_PIN_0_IOMUX                                         (IOMUX_PINCM54)
/* Port definition for Pin Group AIN2 */
#define AIN2_PORT                                                        (GPIOA)

/* Defines for PIN_1: GPIOA.25 with pinCMx 55 on package pin 94 */
#define AIN2_PIN_1_PIN                                          (DL_GPIO_PIN_25)
#define AIN2_PIN_1_IOMUX                                         (IOMUX_PINCM55)
/* Port definition for Pin Group BIN1 */
#define BIN1_PORT                                                        (GPIOA)

/* Defines for PIN_2: GPIOA.26 with pinCMx 59 on package pin 98 */
#define BIN1_PIN_2_PIN                                          (DL_GPIO_PIN_26)
#define BIN1_PIN_2_IOMUX                                         (IOMUX_PINCM59)
/* Port definition for Pin Group BIN2 */
#define BIN2_PORT                                                        (GPIOA)

/* Defines for PIN_3: GPIOA.27 with pinCMx 60 on package pin 99 */
#define BIN2_PIN_3_PIN                                          (DL_GPIO_PIN_27)
#define BIN2_PIN_3_IOMUX                                         (IOMUX_PINCM60)
/* Port definition for Pin Group STBY */
#define STBY_PORT                                                        (GPIOB)

/* Defines for PIN_STBY: GPIOB.0 with pinCMx 12 on package pin 20 */
#define STBY_PIN_STBY_PIN                                        (DL_GPIO_PIN_0)
#define STBY_PIN_STBY_IOMUX                                      (IOMUX_PINCM12)
/* Port definition for Pin Group GPIO_BUZZER */
#define GPIO_BUZZER_PORT                                                 (GPIOA)

/* Defines for PIN_BUZZER: GPIOA.0 with pinCMx 1 on package pin 1 */
#define GPIO_BUZZER_PIN_BUZZER_PIN                               (DL_GPIO_PIN_0)
#define GPIO_BUZZER_PIN_BUZZER_IOMUX                              (IOMUX_PINCM1)
/* Port definition for Pin Group GPIO_LEDS */
#define GPIO_LEDS_PORT                                                   (GPIOA)

/* Defines for USER_LED_BLUE: GPIOA.14 with pinCMx 36 on package pin 53 */
#define GPIO_LEDS_USER_LED_BLUE_PIN                             (DL_GPIO_PIN_14)
#define GPIO_LEDS_USER_LED_BLUE_IOMUX                            (IOMUX_PINCM36)
/* Defines for USER_LED_ORANGE: GPIOA.17 with pinCMx 39 on package pin 69 */
#define GPIO_LEDS_USER_LED_ORANGE_PIN                           (DL_GPIO_PIN_17)
#define GPIO_LEDS_USER_LED_ORANGE_IOMUX                          (IOMUX_PINCM39)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_QEI_Left_init(void);
void SYSCFG_DL_QEI_Right_init(void);
void SYSCFG_DL_UART_0_init(void);
void SYSCFG_DL_UART_1_init(void);
void SYSCFG_DL_UART_2_init(void);
void SYSCFG_DL_DMA_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
