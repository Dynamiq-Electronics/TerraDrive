#include "TerraDrive.h"

// ─── Init ────────────────────────────────────────────────────────────────────
void TerraDrive::init() {
    pinMode(Pins::LEFT_MOTOR_SLEEP,  OUTPUT);
    pinMode(Pins::RIGHT_MOTOR_SLEEP, OUTPUT);

    pinMode(Pins::LEFT_FAULT,  INPUT);
    pinMode(Pins::RIGHT_FAULT, INPUT);

    pinMode(Pins::LIPO_SENSE, INPUT);

    pinMode(Pins::RIGHT_CURRENT_SENSE, INPUT);
    pinMode(Pins::LEFT_CURRENT_SENSE, INPUT);

    pinMode(Pins::DIR_PIN2,  OUTPUT);
    pinMode(Pins::DIR_PIN41, OUTPUT);
    pinMode(Pins::DIR_PIN39, OUTPUT);
    pinMode(Pins::DIR_PIN48, OUTPUT);


    m_pixels.begin();
    _initMCPWM();
    _initADC();
    analogReadResolution(12);
}

void TerraDrive::_initADC() {
    // init ADC2 unit
    adc_oneshot_unit_init_cfg_t unitCfg = {};
    unitCfg.unit_id = ADC_UNIT_2;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unitCfg, &m_adcHandle));

    // channel config
    adc_oneshot_chan_cfg_t chanCfg = {};
    chanCfg.atten    = ADC_ATTEN_DB_0;   // 100mV - 950mV range this allows for motors up to 1.4A to be read. Use 12DB atten for higher amps
    chanCfg.bitwidth = ADC_BITWIDTH_12;

    // Lipo voltage sensor
    adc_oneshot_chan_cfg_t lipoChanCfg = {};
    chanCfg.atten    = ADC_ATTEN_DB_12;
    chanCfg.bitwidth = ADC_BITWIDTH_12;

    // GPIO18 = ADC2 CH7, GPIO14 = ADC2 CH3, GPIO17 = ADC2 CH6
    ESP_ERROR_CHECK(adc_oneshot_config_channel(m_adcHandle, ADC_CHANNEL_7, &chanCfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(m_adcHandle, ADC_CHANNEL_3, &chanCfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(m_adcHandle, ADC_CHANNEL_6, &lipoChanCfg));

    // calibration for left (CH7)
    adc_cali_curve_fitting_config_t caliCfg = {};
    caliCfg.unit_id  = ADC_UNIT_2;
    caliCfg.atten    = ADC_ATTEN_DB_0;
    caliCfg.bitwidth = ADC_BITWIDTH_12;

    caliCfg.chan = ADC_CHANNEL_7;
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&caliCfg, &m_leftCaliHandle));

    caliCfg.chan = ADC_CHANNEL_3;
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&caliCfg, &m_rightCaliHandle));

    caliCfg.atten = ADC_ATTEN_DB_12;
    caliCfg.chan = ADC_CHANNEL_6;
    ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&caliCfg, &m_lipoCaliHandle));

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

float TerraDrive::_readCurrentMilliVolts(adc_channel_t channel, adc_cali_handle_t cali) const {
    int raw = 0;
    int millivolts = 0;
    adc_oneshot_read(m_adcHandle, channel, &raw);
    adc_cali_raw_to_voltage(cali, raw, &millivolts);
    return millivolts;
};
