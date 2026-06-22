#include "actuator_id.h"

#if ACTUATOR_ID_MODE

#include "adc.h"
#include "control.h"
#include "motor.h"
#include "tim.h"
#include "usart.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define ACT_ID_PROTOCOL_VERSION 1U
#define ACT_ID_FRAME_RECORD 1U
#define ACT_ID_PAYLOAD_SIZE 66U
#define ACT_ID_FRAME_SIZE (5U + ACT_ID_PAYLOAD_SIZE + 2U)

#define ACT_ID_EVENT_NONE 0U
#define ACT_ID_EVENT_START 1U
#define ACT_ID_EVENT_PWM_STEP 2U
#define ACT_ID_EVENT_COAST 3U
#define ACT_ID_EVENT_BRAKE 4U
#define ACT_ID_EVENT_COMPLETE 5U
#define ACT_ID_EVENT_ESTOP 100U
#define ACT_ID_EVENT_OVERSPEED 101U
#define ACT_ID_EVENT_UNDERVOLTAGE 103U
#define ACT_ID_EVENT_COMM_TIMEOUT 104U

#define ACT_ID_FAULT_ESTOP (1UL << 0)
#define ACT_ID_FAULT_OVERSPEED (1UL << 1)
#define ACT_ID_FAULT_UNDERVOLTAGE (1UL << 2)
#define ACT_ID_FAULT_COMM_TIMEOUT (1UL << 3)
#define ACT_ID_FAULT_BUFFER_OVERRUN (1UL << 5)
#define ACT_ID_FAULT_CONFIG (1UL << 6)
#define ACT_ID_FAULT_SPINUP_TIMEOUT (1UL << 7)

#define ACT_ID_EXPERIMENT_COAST 1U
#define ACT_ID_EXPERIMENT_PWM_HOLD 2U
#define ACT_ID_EXPERIMENT_PWM_STEP 3U
#define ACT_ID_SIDE_LEFT 1U
#define ACT_ID_SIDE_RIGHT 2U

#if (1000U % ACT_ID_SAMPLE_RATE_HZ) != 0U
#error "ACT_ID_SAMPLE_RATE_HZ must divide the 1 kHz SysTick rate"
#endif

typedef struct {
    uint16_t experiment_id;
    uint8_t experiment;
    uint8_t side;
    int16_t pwm_a;
    int16_t pwm_b;
    int32_t target_speed_mrad_s;
    uint32_t armed_ms;
    uint32_t phase_a_ms;
    uint32_t phase_b_ms;
    uint32_t coast_ms;
    int32_t max_speed_mrad_s;
    uint16_t undervoltage_mv;
} ActuatorIdConfig;

typedef struct {
    uint64_t time_us;
    uint32_t sample_id;
    uint16_t experiment_id;
    uint8_t state;
    uint8_t previous_state;
    uint16_t event_code;
    uint8_t bridge_mode_l;
    uint8_t bridge_mode_r;
    int16_t pwm_cmd_l;
    int16_t pwm_cmd_r;
    int16_t pwm_applied_l;
    int16_t pwm_applied_r;
    uint16_t pwm_reg_l1;
    uint16_t pwm_reg_l2;
    uint16_t pwm_reg_r1;
    uint16_t pwm_reg_r2;
    int32_t encoder_count_l;
    int32_t encoder_count_r;
    int32_t encoder_delta_l;
    int32_t encoder_delta_r;
    uint16_t battery_mv;
    uint32_t fault_flags;
    uint64_t pwm_write_time_us;
} ActuatorIdRecord;

static ActuatorIdConfig config;
static volatile uint8_t config_valid;
static volatile uint8_t initialized;
static volatile uint8_t run_started;
static volatile uint8_t phase_index;
static volatile uint8_t state = ACT_ID_STATE_IDLE;
static volatile uint8_t previous_state = ACT_ID_STATE_IDLE;
static volatile uint16_t pending_event;
static volatile uint32_t fault_flags;
static volatile uint64_t state_start_us;
static volatile uint64_t last_command_us;
static volatile uint64_t pwm_write_time_us;
static volatile uint64_t time_us;
static volatile uint16_t battery_mv;
static volatile int32_t speed_l_mrad_s;
static volatile int32_t speed_r_mrad_s;
static volatile int32_t encoder_count_l;
static volatile int32_t encoder_count_r;
static volatile uint32_t sample_id;

static volatile uint8_t bridge_mode_l = ACT_ID_BRIDGE_OFF;
static volatile uint8_t bridge_mode_r = ACT_ID_BRIDGE_OFF;
static volatile int16_t pwm_cmd_l;
static volatile int16_t pwm_cmd_r;
static volatile int16_t pwm_applied_l;
static volatile int16_t pwm_applied_r;

static uint16_t encoder_raw_l;
static uint16_t encoder_raw_r;
static uint32_t last_cycle;
static uint64_t elapsed_cycles;
static uint8_t sample_divider;
static uint64_t stopped_since_us;

static ActuatorIdRecord record_ring[ACT_ID_RING_CAPACITY];
static volatile uint16_t ring_head;
static volatile uint16_t ring_tail;

static uint8_t tx_frame[ACT_ID_FRAME_SIZE];
static uint16_t tx_length;
static uint16_t tx_position;
static char command_buffer[160];
static uint16_t command_length;

static int32_t abs_i32(int32_t value)
{
    return value < 0 ? -value : value;
}

static int16_t clamp_pwm(long value)
{
    if (value > ACT_ID_PWM_LIMIT) return ACT_ID_PWM_LIMIT;
    if (value < -ACT_ID_PWM_LIMIT) return -ACT_ID_PWM_LIMIT;
    return (int16_t)value;
}

static uint64_t hardware_time_us(void)
{
    uint64_t cycles = elapsed_cycles + (uint32_t)(DWT->CYCCNT - last_cycle);
    return cycles / (ACT_ID_DWT_CLOCK_HZ / 1000000U);
}

static void apply_output(uint8_t mode_l, int16_t command_l,
                         uint8_t mode_r, int16_t command_r)
{
    if (mode_l == bridge_mode_l && mode_r == bridge_mode_r &&
        command_l == pwm_cmd_l && command_r == pwm_cmd_r) return;

    pwm_cmd_l = command_l;
    pwm_cmd_r = command_r;
    pwm_applied_l = 0;
    pwm_applied_r = 0;

    Set_Pwm(mode_l == ACT_ID_BRIDGE_DRIVE ? command_l : 0,
            mode_r == ACT_ID_BRIDGE_DRIVE ? command_r : 0);

    if (mode_l == ACT_ID_BRIDGE_DRIVE) {
        pwm_applied_l = command_l;
    } else if (mode_l == ACT_ID_BRIDGE_COAST) {
        PWMA_IN1 = ACT_ID_COAST_L_IN1;
        PWMA_IN2 = ACT_ID_COAST_L_IN2;
    } else if (mode_l == ACT_ID_BRIDGE_BRAKE) {
        PWMA_IN1 = ACT_ID_BRAKE_L_IN1;
        PWMA_IN2 = ACT_ID_BRAKE_L_IN2;
    } else {
        PWMA_IN1 = 0;
        PWMA_IN2 = 0;
    }

    if (mode_r == ACT_ID_BRIDGE_DRIVE) {
        pwm_applied_r = command_r;
    } else if (mode_r == ACT_ID_BRIDGE_COAST) {
        PWMB_IN1 = ACT_ID_COAST_R_IN1;
        PWMB_IN2 = ACT_ID_COAST_R_IN2;
    } else if (mode_r == ACT_ID_BRIDGE_BRAKE) {
        PWMB_IN1 = ACT_ID_BRAKE_R_IN1;
        PWMB_IN2 = ACT_ID_BRAKE_R_IN2;
    } else {
        PWMB_IN1 = 0;
        PWMB_IN2 = 0;
    }

    bridge_mode_l = mode_l;
    bridge_mode_r = mode_r;
    pwm_write_time_us = hardware_time_us();
}

static void apply_side_drive(int16_t pwm)
{
    uint8_t selected_mode = ACT_ID_BRIDGE_DRIVE;
    if (pwm == 0)
        selected_mode = ACT_ID_ZERO_PWM_BRIDGE_MODE == 1U ?
                        ACT_ID_BRIDGE_COAST : ACT_ID_BRIDGE_BRAKE;
    if (config.side == ACT_ID_SIDE_LEFT) {
        apply_output(selected_mode, pwm, ACT_ID_BRIDGE_COAST, 0);
    } else {
        apply_output(ACT_ID_BRIDGE_COAST, 0, selected_mode, pwm);
    }
}

static void apply_coast(void)
{
    apply_output(ACT_ID_BRIDGE_COAST, 0, ACT_ID_BRIDGE_COAST, 0);
}

static void apply_brake(void)
{
    apply_output(ACT_ID_BRIDGE_BRAKE, 0, ACT_ID_BRIDGE_BRAKE, 0);
}

static void apply_off(void)
{
    apply_output(ACT_ID_BRIDGE_OFF, 0, ACT_ID_BRIDGE_OFF, 0);
}

static void change_state(uint8_t next_state, uint16_t event_code)
{
    previous_state = state;
    state = next_state;
    state_start_us = time_us;
    pending_event = event_code;
    stopped_since_us = 0;
}

static void emergency_stop(uint16_t event_code, uint32_t fault)
{
    fault_flags |= fault;
    run_started = 0;
    change_state(ACT_ID_STATE_E_STOP, event_code);
    apply_off();
}

static int32_t selected_speed(void)
{
    return config.side == ACT_ID_SIDE_LEFT ? speed_l_mrad_s : speed_r_mrad_s;
}

static void finish_run(void)
{
    run_started = 0;
    change_state(ACT_ID_STATE_COMPLETE, ACT_ID_EVENT_COMPLETE);
    apply_coast();
}

static void state_machine_tick(void)
{
    uint64_t elapsed_us;
    int32_t speed;

    if (!run_started) return;

    if (KEY2_STATE) {
        emergency_stop(ACT_ID_EVENT_ESTOP, ACT_ID_FAULT_ESTOP);
        return;
    }
    if (battery_mv != 0U && battery_mv < config.undervoltage_mv) {
        emergency_stop(ACT_ID_EVENT_UNDERVOLTAGE, ACT_ID_FAULT_UNDERVOLTAGE);
        return;
    }

    speed = selected_speed();
    if (abs_i32(speed) > config.max_speed_mrad_s) {
        emergency_stop(ACT_ID_EVENT_OVERSPEED, ACT_ID_FAULT_OVERSPEED);
        return;
    }
    if ((time_us - last_command_us) > ((uint64_t)ACT_ID_COMMAND_TIMEOUT_MS * 1000ULL)) {
        emergency_stop(ACT_ID_EVENT_COMM_TIMEOUT, ACT_ID_FAULT_COMM_TIMEOUT);
        return;
    }

    elapsed_us = time_us - state_start_us;

    if (state == ACT_ID_STATE_ARMED) {
        if (elapsed_us < ((uint64_t)config.armed_ms * 1000ULL)) return;
        if (config.experiment == ACT_ID_EXPERIMENT_COAST) {
            change_state(ACT_ID_STATE_SPINUP, ACT_ID_EVENT_PWM_STEP);
            apply_side_drive(config.pwm_a);
        } else {
            phase_index = 0;
            change_state(ACT_ID_STATE_DRIVE_HOLD, ACT_ID_EVENT_PWM_STEP);
            apply_side_drive(config.pwm_a);
        }
        return;
    }

    if (state == ACT_ID_STATE_SPINUP) {
        if (abs_i32(speed) >= (abs_i32(config.target_speed_mrad_s) * 95L) / 100L) {
            if (stopped_since_us == 0U) stopped_since_us = time_us;
            if ((time_us - stopped_since_us) >= 150000ULL) {
                change_state(ACT_ID_STATE_COAST, ACT_ID_EVENT_COAST);
                apply_coast();
            }
        } else {
            stopped_since_us = 0;
        }
        if (elapsed_us >= ((uint64_t)config.phase_a_ms * 1000ULL)) {
            fault_flags |= ACT_ID_FAULT_SPINUP_TIMEOUT;
            change_state(ACT_ID_STATE_COAST, ACT_ID_EVENT_COAST);
            apply_coast();
        }
        return;
    }

    if (state == ACT_ID_STATE_DRIVE_HOLD) {
        if (phase_index == 0U && elapsed_us >= ((uint64_t)config.phase_a_ms * 1000ULL)) {
            if (config.experiment == ACT_ID_EXPERIMENT_PWM_STEP && config.phase_b_ms > 0U) {
                phase_index = 1U;
                state_start_us = time_us;
                pending_event = ACT_ID_EVENT_PWM_STEP;
                apply_side_drive(config.pwm_b);
            } else {
                change_state(ACT_ID_STATE_COAST, ACT_ID_EVENT_COAST);
                apply_coast();
            }
        } else if (phase_index == 1U &&
                   elapsed_us >= ((uint64_t)config.phase_b_ms * 1000ULL)) {
            change_state(ACT_ID_STATE_COAST, ACT_ID_EVENT_COAST);
            apply_coast();
        }
        return;
    }

    if (state == ACT_ID_STATE_COAST) {
        if (abs_i32(speed) < 500L) {
            if (stopped_since_us == 0U) stopped_since_us = time_us;
            if ((time_us - stopped_since_us) >= 1000000ULL) {
                finish_run();
                return;
            }
        } else {
            stopped_since_us = 0;
        }
        if (elapsed_us >= ((uint64_t)config.coast_ms * 1000ULL)) {
            change_state(ACT_ID_STATE_BRAKE, ACT_ID_EVENT_BRAKE);
            apply_brake();
        }
        return;
    }

    if (state == ACT_ID_STATE_BRAKE && elapsed_us >= 200000ULL) finish_run();
}

static void queue_record(int32_t delta_l, int32_t delta_r)
{
    uint16_t next = (uint16_t)((ring_head + 1U) % ACT_ID_RING_CAPACITY);
    uint32_t current_sample_id = sample_id++;
    ActuatorIdRecord *record;

    if (next == ring_tail) {
        fault_flags |= ACT_ID_FAULT_BUFFER_OVERRUN;
        return;
    }

    record = &record_ring[ring_head];
    record->time_us = time_us;
    record->sample_id = current_sample_id;
    record->experiment_id = config.experiment_id;
    record->state = state;
    record->previous_state = previous_state;
    record->event_code = pending_event;
    record->bridge_mode_l = bridge_mode_l;
    record->bridge_mode_r = bridge_mode_r;
    record->pwm_cmd_l = pwm_cmd_l;
    record->pwm_cmd_r = pwm_cmd_r;
    record->pwm_applied_l = pwm_applied_l;
    record->pwm_applied_r = pwm_applied_r;
    record->pwm_reg_l1 = (uint16_t)PWMA_IN1;
    record->pwm_reg_l2 = (uint16_t)PWMA_IN2;
    record->pwm_reg_r1 = (uint16_t)PWMB_IN1;
    record->pwm_reg_r2 = (uint16_t)PWMB_IN2;
    record->encoder_count_l = encoder_count_l;
    record->encoder_count_r = encoder_count_r;
    record->encoder_delta_l = delta_l;
    record->encoder_delta_r = delta_r;
    record->battery_mv = battery_mv;
    record->fault_flags = fault_flags;
    record->pwm_write_time_us = pwm_write_time_us;
    pending_event = ACT_ID_EVENT_NONE;
    previous_state = state;
    ring_head = next;
}

void ActuatorId_SysTick1ms(void)
{
    uint32_t now_cycle;
    uint16_t now_l;
    uint16_t now_r;
    int16_t raw_delta_l;
    int16_t raw_delta_r;
    int32_t delta_l;
    int32_t delta_r;

    if (!initialized) return;
    now_cycle = DWT->CYCCNT;
    elapsed_cycles += (uint32_t)(now_cycle - last_cycle);
    last_cycle = now_cycle;
    time_us = elapsed_cycles / (ACT_ID_DWT_CLOCK_HZ / 1000000U);

    sample_divider++;
    if (sample_divider < (1000U / ACT_ID_SAMPLE_RATE_HZ)) return;
    sample_divider = 0;

    now_l = (uint16_t)TIM4->CNT;
    now_r = (uint16_t)TIM8->CNT;
    raw_delta_l = (int16_t)(now_l - encoder_raw_l);
    raw_delta_r = (int16_t)(now_r - encoder_raw_r);
    encoder_raw_l = now_l;
    encoder_raw_r = now_r;
    delta_l = (int32_t)raw_delta_l;
    delta_r = -(int32_t)raw_delta_r;
    encoder_count_l += delta_l;
    encoder_count_r += delta_r;

    /* delta * 2*pi*1000 / counts_per_rev / dt, expressed as mrad/s. */
    speed_l_mrad_s = (int32_t)(((int64_t)delta_l * ACT_ID_MRAD_PER_REV * ACT_ID_SAMPLE_RATE_HZ) /
                               ACT_ID_ENCODER_COUNTS_PER_REV);
    speed_r_mrad_s = (int32_t)(((int64_t)delta_r * ACT_ID_MRAD_PER_REV * ACT_ID_SAMPLE_RATE_HZ) /
                               ACT_ID_ENCODER_COUNTS_PER_REV);

    state_machine_tick();
    queue_record(delta_l, delta_r);
}

static uint16_t crc16_ccitt(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;
    uint8_t bit;
    for (i = 0; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (bit = 0; bit < 8U; ++bit)
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1) ^ 0x1021U) : (uint16_t)(crc << 1);
    }
    return crc;
}

static void put_u8(uint8_t **p, uint8_t value) { *(*p)++ = value; }
static void put_u16(uint8_t **p, uint16_t value)
{
    *(*p)++ = (uint8_t)value;
    *(*p)++ = (uint8_t)(value >> 8);
}
static void put_u32(uint8_t **p, uint32_t value)
{
    put_u16(p, (uint16_t)value);
    put_u16(p, (uint16_t)(value >> 16));
}
static void put_u64(uint8_t **p, uint64_t value)
{
    put_u32(p, (uint32_t)value);
    put_u32(p, (uint32_t)(value >> 32));
}

static void encode_record(const ActuatorIdRecord *record)
{
    uint8_t *p = tx_frame;
    uint16_t crc;
    put_u8(&p, 0xA5U);
    put_u8(&p, 0x5AU);
    put_u8(&p, ACT_ID_PROTOCOL_VERSION);
    put_u8(&p, ACT_ID_FRAME_RECORD);
    put_u8(&p, ACT_ID_PAYLOAD_SIZE);
    put_u64(&p, record->time_us);
    put_u32(&p, record->sample_id);
    put_u16(&p, record->experiment_id);
    put_u8(&p, record->state);
    put_u8(&p, record->previous_state);
    put_u16(&p, record->event_code);
    put_u8(&p, record->bridge_mode_l);
    put_u8(&p, record->bridge_mode_r);
    put_u16(&p, (uint16_t)record->pwm_cmd_l);
    put_u16(&p, (uint16_t)record->pwm_cmd_r);
    put_u16(&p, (uint16_t)record->pwm_applied_l);
    put_u16(&p, (uint16_t)record->pwm_applied_r);
    put_u16(&p, record->pwm_reg_l1);
    put_u16(&p, record->pwm_reg_l2);
    put_u16(&p, record->pwm_reg_r1);
    put_u16(&p, record->pwm_reg_r2);
    put_u32(&p, (uint32_t)record->encoder_count_l);
    put_u32(&p, (uint32_t)record->encoder_count_r);
    put_u32(&p, (uint32_t)record->encoder_delta_l);
    put_u32(&p, (uint32_t)record->encoder_delta_r);
    put_u16(&p, record->battery_mv);
    put_u32(&p, record->fault_flags);
    put_u64(&p, record->pwm_write_time_us);
    crc = crc16_ccitt(tx_frame, (uint16_t)(p - tx_frame));
    put_u16(&p, crc);
    tx_length = (uint16_t)(p - tx_frame);
    tx_position = 0;
}

static uint8_t parse_config(char *line)
{
    char *token;
    long values[12];
    uint8_t count = 0;

    token = strtok(line, ",");
    if (token == NULL || strcmp(token, "IDCFG") != 0) return 0;
    while ((token = strtok(NULL, ",")) != NULL && count < 12U)
        values[count++] = strtol(token, NULL, 10);
    if (count != 12U) return 0;
    if (values[1] < ACT_ID_EXPERIMENT_COAST || values[1] > ACT_ID_EXPERIMENT_PWM_STEP) return 0;
    if (values[0] < 1L || values[0] > 65535L) return 0;
    if (values[2] != ACT_ID_SIDE_LEFT && values[2] != ACT_ID_SIDE_RIGHT) return 0;
    if (values[10] <= 0 || values[10] > 200000L) return 0;
    if (values[11] < 6000L || values[11] > 15000L) return 0;
    if (labs(values[3]) > ACT_ID_PWM_LIMIT || labs(values[4]) > ACT_ID_PWM_LIMIT) return 0;
    if (values[6] < 0 || values[7] < 0 || values[8] < 0 || values[9] < 1000L) return 0;

    config.experiment_id = (uint16_t)values[0];
    config.experiment = (uint8_t)values[1];
    config.side = (uint8_t)values[2];
    config.pwm_a = clamp_pwm(values[3]);
    config.pwm_b = clamp_pwm(values[4]);
    config.target_speed_mrad_s = (int32_t)values[5];
    config.armed_ms = (uint32_t)values[6];
    config.phase_a_ms = (uint32_t)values[7];
    config.phase_b_ms = (uint32_t)values[8];
    config.coast_ms = (uint32_t)values[9];
    config.max_speed_mrad_s = (int32_t)values[10];
    config.undervoltage_mv = (uint16_t)values[11];
    return 1;
}

static void handle_command(char *line)
{
    __disable_irq();
    last_command_us = time_us;
    __enable_irq();

    if (strcmp(line, "IDHEART") == 0 || strcmp(line, "IDPING") == 0) return;
    if (strcmp(line, "IDSTOP") == 0) {
        emergency_stop(ACT_ID_EVENT_ESTOP, ACT_ID_FAULT_ESTOP);
        return;
    }
    if (strcmp(line, "IDCOAST") == 0) {
        if (!ACT_ID_BRIDGE_TRUTH_TABLE_CONFIRMED) {
            fault_flags |= ACT_ID_FAULT_CONFIG;
            apply_off();
            return;
        }
        run_started = 0;
        change_state(ACT_ID_STATE_COAST, ACT_ID_EVENT_COAST);
        apply_coast();
        return;
    }
    if (strcmp(line, "IDBRAKE") == 0) {
        if (!ACT_ID_BRIDGE_TRUTH_TABLE_CONFIRMED) {
            fault_flags |= ACT_ID_FAULT_CONFIG;
            apply_off();
            return;
        }
        run_started = 0;
        change_state(ACT_ID_STATE_BRAKE, ACT_ID_EVENT_BRAKE);
        apply_brake();
        return;
    }
    if (strcmp(line, "IDSTART") == 0) {
        if (config_valid && state == ACT_ID_STATE_ARMED && ACT_ID_BRIDGE_TRUTH_TABLE_CONFIRMED) {
            __disable_irq();
            ring_tail = ring_head;
            sample_id = 0;
            encoder_count_l = 0;
            encoder_count_r = 0;
            encoder_raw_l = (uint16_t)TIM4->CNT;
            encoder_raw_r = (uint16_t)TIM8->CNT;
            run_started = 1;
            last_command_us = time_us;
            state_start_us = time_us;
            pending_event = ACT_ID_EVENT_START;
            __enable_irq();
        }
        return;
    }
    if (strncmp(line, "IDCFG,", 6) == 0) {
        if (!ACT_ID_BRIDGE_TRUTH_TABLE_CONFIRMED) {
            fault_flags |= ACT_ID_FAULT_CONFIG;
            apply_off();
            return;
        }
        if (parse_config(line)) {
            __disable_irq();
            ring_tail = ring_head;
            sample_id = 0;
            encoder_count_l = 0;
            encoder_count_r = 0;
            encoder_raw_l = (uint16_t)TIM4->CNT;
            encoder_raw_r = (uint16_t)TIM8->CNT;
            fault_flags = 0;
            config_valid = 1;
            run_started = 0;
            phase_index = 0;
            change_state(ACT_ID_STATE_ARMED, ACT_ID_EVENT_NONE);
            apply_coast();
            __enable_irq();
        } else {
            fault_flags |= ACT_ID_FAULT_CONFIG;
        }
    }
}

static void receive_commands(void)
{
    while ((USART1->SR & USART_SR_RXNE) != 0U) {
        char ch = (char)(USART1->DR & 0xFFU);
        if (ch == '\r') continue;
        if (ch == '\n') {
            command_buffer[command_length] = '\0';
            if (command_length > 0U) handle_command(command_buffer);
            command_length = 0;
        } else if (command_length < (sizeof(command_buffer) - 1U)) {
            command_buffer[command_length++] = ch;
        } else {
            command_length = 0;
            fault_flags |= ACT_ID_FAULT_CONFIG;
        }
    }
}

static void transmit_records(void)
{
    uint8_t burst = 0;
    while (burst++ < 8U) {
        if (tx_position < tx_length) {
            if ((USART1->SR & USART_SR_TXE) == 0U) return;
            USART1->DR = tx_frame[tx_position++];
            continue;
        }
        if (ring_tail == ring_head) return;
        encode_record(&record_ring[ring_tail]);
        ring_tail = (uint16_t)((ring_tail + 1U) % ACT_ID_RING_CAPACITY);
    }
}

void ActuatorId_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    last_cycle = DWT->CYCCNT;
    elapsed_cycles = 0;
    time_us = 0;
    encoder_raw_l = (uint16_t)TIM4->CNT;
    encoder_raw_r = (uint16_t)TIM8->CNT;
    bridge_mode_l = ACT_ID_BRIDGE_COAST; /* force apply_off() to write registers */
    bridge_mode_r = ACT_ID_BRIDGE_COAST;
    Control_ResetIncrementalPI();
    apply_off();
    initialized = 1;
}

void ActuatorId_Process(void)
{
    static uint32_t last_battery_ms;
    uint32_t now_ms = HAL_GetTick();
    receive_commands();
    transmit_records();
    if ((now_ms - last_battery_ms) >= 20U) {
        last_battery_ms = now_ms;
        battery_mv = (uint16_t)(Get_battery_volt() * 10);
    }
}

#endif
