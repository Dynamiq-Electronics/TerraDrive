#pragma once
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <driver/mcpwm_prelude.h>

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

namespace Pins {
    constexpr int LIPO_SENSE{17}; // AD2 CH6

    constexpr int LEFT_CURRENT_SENSE{18};   // ADC2 CH7
    constexpr int RIGHT_CURRENT_SENSE{14};  // ADC2 CH3

    constexpr int LEFT_MOTOR_SLEEP{8};
    constexpr int RIGHT_MOTOR_SLEEP{12};

    constexpr int LEFT_FAULT{15};
    constexpr int RIGHT_FAULT{13};

    constexpr int LEFT_IN1{9};
    constexpr int LEFT_IN2{10};

    constexpr int RIGHT_IN1{47};
    constexpr int RIGHT_IN2{21};

    constexpr int NEO_PIN{11};

    constexpr int DIR_PIN2{1};
    constexpr int DIR_PIN41{42};
    constexpr int DIR_PIN39{40};
    constexpr int DIR_PIN48{38};
}

namespace MotorCfg {
    // Center-aligned: f = resolution / (2 * period)
    // 80MHz / (2 * 10000) = 4kHz, 10000 duty levels (~13.3 bit)
    constexpr uint32_t PWM_RESOLUTION_HZ = 80'000'000;
    constexpr uint32_t PWM_FREQ_HZ       = 4000;
    constexpr uint32_t PWM_PERIOD_TICKS  = PWM_RESOLUTION_HZ / PWM_FREQ_HZ; // 10000
    constexpr uint32_t PWM_PEAK_TICKS = MotorCfg::PWM_PERIOD_TICKS / 2; // 10000

    constexpr float SENSE_RESISTOR = 1500.0f;
}

enum class Pins5v {
    PIN2  = 2,
    PIN41 = 41,
    PIN39 = 39,
    PIN48 = 48
};

struct PinMapping { int gpio; int dirPin; };
constexpr PinMapping PIN_MAPPINGS[] = {
    {2,  Pins::DIR_PIN2  },
    {41, Pins::DIR_PIN41 },
    {39, Pins::DIR_PIN39 },
    {48, Pins::DIR_PIN48 },
};

// Given a pinout port, return the DIR logic pin.
constexpr int getDirPin(int gpio) {
    for (const auto& m : PIN_MAPPINGS)
        if (m.gpio == gpio) return m.dirPin;
    return -1;
}

class TerraDrive {
    public:
        TerraDrive() = default;
        void init();

        void setEnableMotors(bool enable);
        void setLeftMotor(float output);
        void setRightMotor(float output);
        void pinMode5V(Pins5v pin, uint8_t state);

        float getLipoVoltage() const { return _readCurrentMilliVolts(adc_channel_t::ADC_CHANNEL_6, m_lipoCaliHandle) * 11.0f / 1000.0f; }

        // The formula for the motor current is given by V = I_motor * 45 * 15 / 10^3
        float getLeftCurrentRaw()  const { return _readCurrentMilliVolts(adc_channel_t::ADC_CHANNEL_7, m_leftCaliHandle) / 675.0f; }
        float getRightCurrentRaw() const { return _readCurrentMilliVolts(adc_channel_t::ADC_CHANNEL_3, m_leftCaliHandle) / 675.0f; }

        bool isFaulted();

        Adafruit_NeoPixel& getNeoPixel() { return m_pixels; }

    private:
        float _readCurrentMilliVolts(adc_channel_t channel, adc_cali_handle_t cali) const;
        void _initMCPWM();
        void _initADC();


        void _setMotorOutput(mcpwm_cmpr_handle_t cmpr,
                                  mcpwm_gen_handle_t  gen1,
                                  mcpwm_gen_handle_t  gen2,
                                  float output);

        // MCPWM 
        mcpwm_timer_handle_t m_pwmTimer = NULL;
        mcpwm_oper_handle_t m_leftOper = NULL;
        mcpwm_oper_handle_t m_rightOper = NULL;
        mcpwm_cmpr_handle_t m_leftCmpr = NULL;
        mcpwm_cmpr_handle_t m_rightCmpr = NULL;
        mcpwm_gen_handle_t m_rightGenFwd = NULL; // forward PWM signal
        mcpwm_gen_handle_t m_leftGenFwd = NULL;
        mcpwm_gen_handle_t m_rightGenRvs = NULL; // Reverse PWM signal
        mcpwm_gen_handle_t m_leftGenRvs = NULL;

        // 
        adc_oneshot_unit_handle_t m_adcHandle;
        adc_cali_handle_t m_leftCaliHandle;
        adc_cali_handle_t m_rightCaliHandle;
        adc_cali_handle_t m_lipoCaliHandle;

        // ADC filter
        float m_leftFiltered = 0.0f;
        float m_rightFiltered = 0.0f;
        static constexpr float ALPHA = 0.15f;  // lower = smoother, slower

        static constexpr int NUM_PIXELS{2};
        Adafruit_NeoPixel m_pixels{NUM_PIXELS, Pins::NEO_PIN, NEO_GRB + NEO_KHZ800};
};