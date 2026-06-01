#include "TerraDrive.h"

// ─── Init ────────────────────────────────────────────────────────────────────
void TerraDrive::init() {
    pinMode(Pins::LEFT_MOTOR_SLEEP,  OUTPUT);
    pinMode(Pins::RIGHT_MOTOR_SLEEP, OUTPUT);

    pinMode(Pins::LEFT_FAULT,  INPUT);
    pinMode(Pins::RIGHT_FAULT, INPUT);

    pinMode(Pins::LIPO_SENSE, INPUT);

    pinMode(Pins::DIR_PIN2,  OUTPUT);
    pinMode(Pins::DIR_PIN41, OUTPUT);
    pinMode(Pins::DIR_PIN39, OUTPUT);
    pinMode(Pins::DIR_PIN48, OUTPUT);

    m_pixels.begin();
    _initMCPWM();
}

void TerraDrive::_initMCPWM() {
    // Config class for the PWM timer
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MotorCfg::PWM_RESOLUTION_HZ,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN, 
        .period_ticks = MotorCfg::PWM_PERIOD_TICKS
    };

    // Create the new timer with the above configs.
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &m_pwmTimer));

    // operator config
    mcpwm_operator_config_t oper_config = {
        .group_id = 0
    };
    // initialise operators.
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &m_leftOper));
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_config, &m_rightOper));

    // connect both operators to the same timer
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(m_leftOper, m_pwmTimer));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(m_rightOper, m_pwmTimer));

    // comparator config
    mcpwm_comparator_config_t cmpr_config = {};
    cmpr_config.flags.update_cmp_on_tez = true; // update the new compare value when counter reaches 0
    // connect both operators with their comparators
    ESP_ERROR_CHECK(mcpwm_new_comparator(m_leftOper, &cmpr_config, &m_leftCmpr));
    ESP_ERROR_CHECK(mcpwm_new_comparator(m_rightOper, &cmpr_config, &m_rightCmpr));

    // generator pins configurations
    mcpwm_generator_config_t genLeftFwd_config = {
        .gen_gpio_num = Pins::LEFT_IN1
    };
    mcpwm_generator_config_t genLeftRvs_config = {
        .gen_gpio_num = Pins::LEFT_IN2
    };
    mcpwm_generator_config_t genRightFwd_config = {
        .gen_gpio_num = Pins::RIGHT_IN1
    };
    mcpwm_generator_config_t genRightRvs_config = {
        .gen_gpio_num = Pins::RIGHT_IN2
    };

    // create the generators
    ESP_ERROR_CHECK(mcpwm_new_generator(m_leftOper, &genLeftFwd_config, &m_leftGenFwd));
    ESP_ERROR_CHECK(mcpwm_new_generator(m_leftOper, &genLeftRvs_config, &m_leftGenRvs));
    ESP_ERROR_CHECK(mcpwm_new_generator(m_rightOper, &genRightFwd_config, &m_rightGenFwd));
    ESP_ERROR_CHECK(mcpwm_new_generator(m_rightOper, &genRightRvs_config, &m_rightGenRvs));

    // Generator actions on compare events.
    // Left forward generator — PWM active
    mcpwm_generator_set_action_on_compare_event(m_leftGenFwd,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, m_leftCmpr, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(m_leftGenFwd,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, m_leftCmpr, MCPWM_GEN_ACTION_LOW));

    // Left reverse generator — PWM active
    mcpwm_generator_set_action_on_compare_event(m_leftGenRvs,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, m_leftCmpr, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(m_leftGenRvs,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, m_leftCmpr, MCPWM_GEN_ACTION_LOW));

    // Same for right
    mcpwm_generator_set_action_on_compare_event(m_rightGenFwd,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, m_rightCmpr, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(m_rightGenFwd,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, m_rightCmpr, MCPWM_GEN_ACTION_LOW));

    mcpwm_generator_set_action_on_compare_event(m_rightGenRvs,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, m_rightCmpr, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(m_rightGenRvs,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, m_rightCmpr, MCPWM_GEN_ACTION_LOW));

    // start the pwm timer
    mcpwm_timer_enable(m_pwmTimer);
    mcpwm_timer_start_stop(m_pwmTimer, MCPWM_TIMER_START_NO_STOP);
}

// ─── update() ────────────────────────────────────────────────────────────────
void TerraDrive::update() {
    // if (!m_sampleFlag) return;
    // m_sampleFlag = false;

    // int rawLeft = 0, rawRight = 0, lipo = 0;
    // adc_oneshot_read(m_adcHandle, ADC_CHANNEL_7, &rawLeft);
    // adc_oneshot_read(m_adcHandle, ADC_CHANNEL_3, &rawRight);
    // adc_oneshot_read(m_adcHandle, ADC_CHANNEL_6, &lipo);

    // m_lipoVoltage =  (lipo / 4095.0f) * 36.3f; // 36.3 -> 3v3 voltage divider math
    // // m_leftCurrentA  = _rawToAmps(rawLeft);
    // // m_rightCurrentA = _rawToAmps(rawRight);

    // float rawLeftA = _rawToAmps(rawLeft);
    // m_leftFiltered += ALPHA * (rawLeftA - m_leftFiltered);
    // float rawRightA = _rawToAmps(rawRight);
    // m_rightFiltered += ALPHA * (rawRightA - m_rightFiltered);

}

// float TerraDrive::_rawToAmps(int raw) const {
//     float voltage = (raw / 4095.0f) * 3.3f;
//     return voltage * 1000.0f / MotorCfg::SENSE_RESISTOR;
// }

// ─── Motor control ────────────────────────────────────────────────────────────
void TerraDrive::_setMotorOutput(mcpwm_cmpr_handle_t cmpr,
                                  mcpwm_gen_handle_t  gen1,
                                  mcpwm_gen_handle_t  gen2,
                                  float output) {
    output = constrain(output, -100.0f, 100.0f);

    // Compare value relative to peak (0–10000)
    uint32_t duty = static_cast<uint32_t>(
        (fabsf(output) / 100.0f) * MotorCfg::PWM_PEAK_TICKS
    );
    // Clamp so compare never lands on 0 (boundary with counter zero)
    if (duty == 0) duty = 1;

    if (output > 0.0f) {
        mcpwm_comparator_set_compare_value(cmpr, duty);
        mcpwm_generator_set_force_level(gen1, -1, true);  // release — PWM active
        mcpwm_generator_set_force_level(gen2,  1, true);  // force HIGH
    } else if (output < 0.0f) {
        mcpwm_comparator_set_compare_value(cmpr, duty);
        mcpwm_generator_set_force_level(gen1,  1, true);  // force HIGH
        mcpwm_generator_set_force_level(gen2, -1, true);  // release — PWM active
    } else {
        mcpwm_generator_set_force_level(gen1, 1, true);   // both forced HIGH = BRAKE
        mcpwm_generator_set_force_level(gen2, 1, true);
    }
}


void TerraDrive::setLeftMotor(float output) {
    // Left motor wired inverted — negate to normalise direction
    _setMotorOutput(m_leftCmpr, m_leftGenFwd,  m_leftGenRvs, output);
}
void TerraDrive::setRightMotor(float output) {
    // Left motor wired inverted — negate to normalise direction
    _setMotorOutput(m_rightCmpr, m_rightGenFwd,  m_rightGenRvs, output);
}

// // ─── Enable / Fault ───────────────────────────────────────────────────────────
void TerraDrive::setEnableMotors(bool enable) {
    if (!enable) {
        _setMotorOutput(m_leftCmpr, m_leftGenFwd,  m_leftGenRvs, 0);
        _setMotorOutput(m_rightCmpr, m_rightGenFwd,  m_rightGenRvs, 0);
    }
    digitalWrite(Pins::LEFT_MOTOR_SLEEP,  enable);
    digitalWrite(Pins::RIGHT_MOTOR_SLEEP, enable);
}

bool TerraDrive::isFaulted() {
    return !digitalRead(Pins::LEFT_FAULT) || !digitalRead(Pins::RIGHT_FAULT);
}

// ─── 5V level shifter ────────────────────────────────────────────────────────
void TerraDrive::pinMode5V(Pins5v pin, uint8_t state) {
    pinMode(static_cast<int>(pin), state);
    if (state == OUTPUT) {
        digitalWrite(getDirPin(static_cast<int>(pin)), HIGH);
    } else if (state == INPUT) {
        digitalWrite(getDirPin(static_cast<int>(pin)), LOW);
    }
}