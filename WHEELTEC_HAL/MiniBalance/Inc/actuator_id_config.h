#ifndef __ACTUATOR_ID_CONFIG_H
#define __ACTUATOR_ID_CONFIG_H

/*
 * Dedicated open-loop actuator-identification firmware build.
 * Set to 0 to restore the normal balance controller.
 */
#define ACTUATOR_ID_MODE 1

/* 500 Hz is sustainable on the verified USART1/ST-Link path at 460800 baud. */
#define ACT_ID_SAMPLE_RATE_HZ 500U
#define ACT_ID_UART_BAUD 460800U

/* SystemClock_Config() sets STM32F103 SYSCLK/HCLK to 72 MHz in this project. */
#define ACT_ID_DWT_CLOCK_HZ 72000000UL

/* First-rig safety limit (20% of the normal 6900-count limit). */
#define ACT_ID_PWM_LIMIT 1380
#define ACT_ID_ABSOLUTE_PWM_LIMIT 6900
#define ACT_ID_ENCODER_COUNTS_PER_REV 60000L
#define ACT_ID_MRAD_PER_REV 6283L
#define ACT_ID_UNDERVOLTAGE_DEFAULT_MV 11100U
#define ACT_ID_COMMAND_TIMEOUT_MS 1500U
#define ACT_ID_RING_CAPACITY 64U

/* Explicit bridge state used for a zero platform in pwmstep: 1=COAST, 2=BRAKE. */
#define ACT_ID_ZERO_PWM_BRIDGE_MODE 1U

/*
 * Verified against ref/ C10A schematic + AT8236 datasheet:
 * IN1=0,IN2=0 -> OUT1=Z,OUT2=Z (COAST/sleep after 1 ms)
 * IN1=1,IN2=1 -> OUT1=L,OUT2=L (BRAKE)
 * See WHEELTEC_HAL/ACTUATOR_ID.md for exact evidence paths.
 */
#define ACT_ID_BRIDGE_TRUTH_TABLE_CONFIRMED 1

#define ACT_ID_PWM_LOW  0U
#define ACT_ID_PWM_HIGH 7200U

#define ACT_ID_COAST_L_IN1 ACT_ID_PWM_LOW
#define ACT_ID_COAST_L_IN2 ACT_ID_PWM_LOW
#define ACT_ID_COAST_R_IN1 ACT_ID_PWM_LOW
#define ACT_ID_COAST_R_IN2 ACT_ID_PWM_LOW

#define ACT_ID_BRAKE_L_IN1 ACT_ID_PWM_HIGH
#define ACT_ID_BRAKE_L_IN2 ACT_ID_PWM_HIGH
#define ACT_ID_BRAKE_R_IN1 ACT_ID_PWM_HIGH
#define ACT_ID_BRAKE_R_IN2 ACT_ID_PWM_HIGH

#if ACT_ID_ZERO_PWM_BRIDGE_MODE != 1U && ACT_ID_ZERO_PWM_BRIDGE_MODE != 2U
#error "ACT_ID_ZERO_PWM_BRIDGE_MODE must be 1 (COAST) or 2 (BRAKE)"
#endif

#endif
