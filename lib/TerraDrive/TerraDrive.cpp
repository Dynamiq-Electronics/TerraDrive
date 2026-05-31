#include "TerraDrive.h"

// ─── ISR ────────────────────────────────────────────────────────────────────
// Fires at the PEAK of the triangle wave = center of PWM on-time
// Current is at its average value here — ideal sample point
bool IRAM_ATTR TerraDrive::_onTimerPeak(mcpwm_timer_handle_t,
                                         const mcpwm_timer_event_data_t*,
                                         void* ctx) {
    static_cast<TerraDrive*>(ctx)->m_sampleFlag = true;
    return false;  // no task yield
}

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

    _initMCPWM();
    _initADC();

    m_pixels.begin();
}

void TerraDrive::_initMCPWM() {
    // 1. Timer — up-down (center-aligned), 80MHz resolution, 4kHz
    //    period_ticks = full cycle count; driver sets peak = period_ticks / 2
    //    80MHz / 4kHz = 20000 → peak = 10000, counter: 0 → 10000 → 0
    mcpwm_timer_config_t timerCfg = {
        .group_id      = 0,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = MotorCfg::PWM_RESOLUTION_HZ,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP_DOWN,
        .period_ticks  = MotorCfg::PWM_PERIOD_TICKS,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timerCfg, &m_pwmTimer));

    // Register peak (FULL) event callback — this is our ADC trigger point
    mcpwm_timer_event_callbacks_t timerCbs = {
        .on_full  = _onTimerPeak,
        .on_empty = nullptr,
        .on_stop  = nullptr,
    };
    ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(m_pwmTimer, &timerCbs, this));

    // 2. Operators — one per motor, both sync'd to the same timer
    mcpwm_operator_config_t operCfg = { .group_id = 0 };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operCfg, &m_leftOper));
    ESP_ERROR_CHECK(mcpwm_new_operator(&operCfg, &m_rightOper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(m_leftOper,  m_pwmTimer));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(m_rightOper, m_pwmTimer));

    // 3. Comparators — one per motor pin (4 total)
    mcpwm_comparator_config_t cmprCfg = {
        .flags = { .update_cmp_on_tez = true }
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(m_leftOper,  &cmprCfg, &m_leftCmpr1));
    ESP_ERROR_CHECK(mcpwm_new_comparator(m_leftOper,  &cmprCfg, &m_leftCmpr2));
    ESP_ERROR_CHECK(mcpwm_new_comparator(m_rightOper, &cmprCfg, &m_rightCmpr1));
    ESP_ERROR_CHECK(mcpwm_new_comparator(m_rightOper, &cmprCfg, &m_rightCmpr2));

    // Initial compare values don't matter — force level overrides everything
    mcpwm_comparator_set_compare_value(m_leftCmpr1,  0);
    mcpwm_comparator_set_compare_value(m_leftCmpr2,  0);
    mcpwm_comparator_set_compare_value(m_rightCmpr1, 0);
    mcpwm_comparator_set_compare_value(m_rightCmpr2, 0);

    // 4. Generators — bind comparators to GPIO pins
    _initGenerator(m_leftGen1,  m_leftOper,  m_leftCmpr1,  Pins::LEFT_IN1);
    _initGenerator(m_leftGen2,  m_leftOper,  m_leftCmpr2,  Pins::LEFT_IN2);
    _initGenerator(m_rightGen1, m_rightOper, m_rightCmpr1, Pins::RIGHT_IN1);
    _initGenerator(m_rightGen2, m_rightOper, m_rightCmpr2, Pins::RIGHT_IN2);

    // 5. Force all outputs LOW before starting timer — clean startup
    mcpwm_generator_set_force_level(m_leftGen1,  0, true);
    mcpwm_generator_set_force_level(m_leftGen2,  0, true);
    mcpwm_generator_set_force_level(m_rightGen1, 0, true);
    mcpwm_generator_set_force_level(m_rightGen2, 0, true);

    // 6. Start timer
    ESP_ERROR_CHECK(mcpwm_timer_enable(m_pwmTimer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(m_pwmTimer, MCPWM_TIMER_START_NO_STOP));
}

// Center-aligned generator actions (compare events only):
//   Counting UP,   compare match → HIGH  (symmetric rising edge)
//   Counting DOWN, compare match → LOW   (symmetric falling edge)
// Force level handles the "off" state — no timer event actions needed.
void TerraDrive::_initGenerator(mcpwm_gen_handle_t&  gen,
                                 mcpwm_oper_handle_t  oper,
                                 mcpwm_cmpr_handle_t  cmpr,
                                 int                  gpio) {
    mcpwm_generator_config_t genCfg = { .gen_gpio_num = gpio };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &genCfg, &gen));

    // Compare match going UP → HIGH
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_UP,
            cmpr,
            MCPWM_GEN_ACTION_HIGH
        )
    ));

    // Compare match going DOWN → LOW
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        gen,
        MCPWM_GEN_COMPARE_EVENT_ACTION(
            MCPWM_TIMER_DIRECTION_DOWN,
            cmpr,
            MCPWM_GEN_ACTION_LOW
        )
    ));
}

void TerraDrive::_initADC() {
    adc_oneshot_unit_init_cfg_t unitCfg = {
        .unit_id  = ADC_UNIT_2,
        .clk_src  = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unitCfg, &m_adcHandle));

    adc_oneshot_chan_cfg_t chanCfg = {
        .atten     = ADC_ATTEN_DB_12,
        .bitwidth  = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(m_adcHandle, ADC_CHANNEL_7, &chanCfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(m_adcHandle, ADC_CHANNEL_3, &chanCfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(m_adcHandle, ADC_CHANNEL_6, &chanCfg));
}

// ─── update() ────────────────────────────────────────────────────────────────
void TerraDrive::update() {
    if (!m_sampleFlag) return;
    m_sampleFlag = false;

    int rawLeft = 0, rawRight = 0, lipo = 0;
    adc_oneshot_read(m_adcHandle, ADC_CHANNEL_7, &rawLeft);
    adc_oneshot_read(m_adcHandle, ADC_CHANNEL_3, &rawRight);
    adc_oneshot_read(m_adcHandle, ADC_CHANNEL_6, &lipo);

    m_lipoVoltage =  (lipo / 4095.0f) * 36.3f; // 36.3 -> 3v3 voltage divider math
    // m_leftCurrentA  = _rawToAmps(rawLeft);
    // m_rightCurrentA = _rawToAmps(rawRight);

    float rawLeftA = _rawToAmps(rawLeft);
    m_leftFiltered += ALPHA * (rawLeftA - m_leftFiltered);
    float rawRightA = _rawToAmps(rawRight);
    m_rightFiltered += ALPHA * (rawRightA - m_rightFiltered);

}

float TerraDrive::_rawToAmps(int raw) const {
    float voltage = (raw / 4095.0f) * 3.3f;
    return voltage * 1000.0f / MotorCfg::SENSE_RESISTOR;
}

// ─── Motor control ────────────────────────────────────────────────────────────
void TerraDrive::_setMotorOutput(mcpwm_cmpr_handle_t cmpr1,
                                  mcpwm_cmpr_handle_t cmpr2,
                                  mcpwm_gen_handle_t  gen1,
                                  mcpwm_gen_handle_t  gen2,
                                  float output) {
    output = constrain(output, -100.0f, 100.0f);

    // Compare value relative to peak (0–10000)
    // Lower compare → wider pulse → higher duty
    uint32_t duty = static_cast<uint32_t>(
        (fabsf(output) / 100.0f) * MotorCfg::PWM_PEAK_TICKS
    );
    // Clamp so compare never lands on 0 (boundary with counter zero)
    if (duty == 0) duty = 1;

    if (output > 0.0f) {
        mcpwm_comparator_set_compare_value(cmpr1, duty);
        mcpwm_generator_set_force_level(gen1, -1, true);  // release — PWM active
        mcpwm_generator_set_force_level(gen2,  1, true);  // force HIGH
    } else if (output < 0.0f) {
        mcpwm_comparator_set_compare_value(cmpr2, duty);
        mcpwm_generator_set_force_level(gen1,  1, true);  // force HIGH
        mcpwm_generator_set_force_level(gen2, -1, true);  // release — PWM active
    } else {
        mcpwm_generator_set_force_level(gen1, 1, true);   // both forced HIGH = BRAKE
        mcpwm_generator_set_force_level(gen2, 1, true);
    }

}

void TerraDrive::setLeftMotor(float output) {
    // Left motor wired inverted — negate to normalise direction
    _setMotorOutput(m_leftCmpr1, m_leftCmpr2,
                    m_leftGen1,  m_leftGen2, -output);
}

void TerraDrive::setRightMotor(float output) {
    _setMotorOutput(m_rightCmpr1, m_rightCmpr2,
                    m_rightGen1,  m_rightGen2, output);
}

// ─── Enable / Fault ───────────────────────────────────────────────────────────
void TerraDrive::setEnableMotors(bool enable) {
    if (!enable) {
        _setMotorOutput(m_leftCmpr1,  m_leftCmpr2,
                        m_leftGen1,   m_leftGen2,  0);
        _setMotorOutput(m_rightCmpr1, m_rightCmpr2,
                        m_rightGen1,  m_rightGen2, 0);
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