#include <cstdint>
#include <cmath>   // For sin, M_PI
#include <utility> // For std::pair
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "ESP32Waveu.h"

using tinyalg::waveu::PhaseGenerator;
using tinyalg::waveu::LUTIndexFunction;
using tinyalg::waveu::LUTHelper;
using tinyalg::waveu::LUTSize;
using tinyalg::waveu::lut_type_t;
using tinyalg::waveu::LUT_1024;

static const char* TAG = "UserWaveConfig";

/// @brief LUT : Select an LUT size from the predefined LUTSize's.
lut_type_t lut[LUT_1024];

// Derived class for user wave arguments
class UserWaveArgs : public tinyalg::waveu::WaveConfigArgs {
public:
    float f_start;
    float f_end;
    float t_duration;

    UserWaveArgs(float f_start, float f_end, float t_duration)
        : f_start(f_start), f_end(f_end), t_duration(t_duration) {}
};

class UserWaveConfig : public tinyalg::waveu::WaveConfigInterface {
protected:
    virtual lut_type_t fill_function_sine(LUTSize size, size_t index, float amplitude = 127.5, float offset = 127.5, float delta = 0) {
        size_t table_size = static_cast<size_t>(size);
        return static_cast<lut_type_t>(
            amplitude * sin(index * 2 * M_PI / (table_size - 1) + delta) + offset + 0.5
        );
    }

    virtual void initializePhaseGenerator(uint32_t sampleRate) {
        if (_phaseGeneratorL == nullptr) {
            _phaseGeneratorL = new PhaseGenerator(sampleRate);
        }
        if (_phaseGeneratorE == nullptr) {
            _phaseGeneratorE = new PhaseGenerator(sampleRate);
        }
    }

    virtual void setPhaseGeneratorFrequency(float frequency) {
        _phaseGeneratorL->setFrequency(frequency);
        _phaseGeneratorE->setFrequency(frequency);
        ESP_LOGI(TAG, "Frequency set to %.02fHz and %.02fHz", _phaseGeneratorL->getFrequency(), _phaseGeneratorE->getFrequency());
    }

    virtual void populateLUTs() {
        size_t table_size = static_cast<size_t>(_lutSize);
        for (size_t i = 0; i < table_size; ++i) {
            lut[i] = fill_function_sine(_lutSize, i);
        }
    }

public:
    ~UserWaveConfig() override {
        delete _phaseGeneratorL; // Clean up
        delete _phaseGeneratorE; // Clean up
    }

    void initialize(uint32_t sampleRate) override {
        // Initializes the PhaseGenerator with the specified sample rate.
        initializePhaseGenerator(sampleRate);

        // Retrieves a function to calculate the LUT index from a phase.
        _getIndex = LUTHelper::getIndexFunction(_lutSize);
    }

    void configure(const tinyalg::waveu::WaveConfigArgs& args) override {
        const auto& waveArgs = dynamic_cast<const UserWaveArgs&>(args); // Safe because caller ensures type

        _f_start = waveArgs.f_start;
        _f_end = waveArgs.f_end;
        _t_duration = waveArgs.t_duration;

        // Sets the desired frequency and calculates the phase increment.
        setPhaseGeneratorFrequency(waveArgs.f_start);

        // Fills the values to the LUT using the supplied function.
        populateLUTs();
    }

    void prepareCycle(double elapsedTime) override {
        // Linear sweep
        float nextFrequencyL = _f_start + (_f_end - _f_start) * elapsedTime / _t_duration;
        if (nextFrequencyL > _f_end) {
            nextFrequencyL = _f_end;
        }
        _phaseGeneratorL->setFrequency(nextFrequencyL);

        // Exponential sweep
        float nextFrequencyE = _f_start * std::pow(_f_end / _f_start, elapsedTime / _t_duration);
        if (nextFrequencyE > _f_end) {
            nextFrequencyE = _f_end;
        }
        _phaseGeneratorE->setFrequency(nextFrequencyE);
    }

    uint8_t nextSample() override {
        // Step 1: Advance the phase to the next position.
        _phaseGeneratorL->updatePhase();

        // Step 2: Retrieve the updated phase value.
        uint32_t currentPhase = _phaseGeneratorL->getPhase();

        // Step 3: Map the phase value to an appropriate index in the lookup table.
        int lutIndex = _getIndex(currentPhase, PhaseGenerator::N_BITS);

        // Step 4: Fetch the corresponding voltage value (0-255) from the lookup table using the macro.
        lut_type_t digi_val = GET_LUT_VALUE(lut, lutIndex);

        // Return the voltage value for the sample.
        return (uint8_t)digi_val;
    }

#ifdef CONFIG_WAVEU_CHANNEL_MODE_ALTER
    uint8_t nextSampleB() override {
        // Step 1: Advance the phase to the next position.
        _phaseGeneratorE->updatePhase();

        // Step 2: Retrieve the updated phase value.
        uint32_t currentPhase = _phaseGeneratorE->getPhase();

        // Step 3: Map the phase value to an appropriate index in the lookup table.
        int lutIndex = _getIndex(currentPhase, PhaseGenerator::N_BITS);

        // Step 4: Fetch the corresponding voltage value (0-255) from the lookup table using the macro.
        lut_type_t digi_val = GET_LUT_VALUE(lut, lutIndex);

        // Return the voltage value for the sample.
        return (uint8_t)digi_val;
    }
#endif

    void reset() override {
    }

private:
    /// @brief Initial frequency
    float _f_start;

    /// @brief Final frequency
    float _f_end;

    /// @brief Total sweep duration
    float _t_duration;

    /// @brief Pointer to PhaseGenerator for linear sweep
    PhaseGenerator* _phaseGeneratorL = nullptr;

    /// @brief Pointer to PhaseGenerator for exponential sweep
    PhaseGenerator* _phaseGeneratorE = nullptr;

    /// @brief Function pointer to calculate the LUT index 
    LUTIndexFunction _getIndex = nullptr;

    /// @brief LUT size
    LUTSize _lutSize = LUT_1024;
};

extern "C" {
    void app_main(void)
    {
        // Initialize the waveform generator with a user-defined configuration.
        tinyalg::waveu::ESP32Waveu<UserWaveConfig> waveu;

        float f_start = 100.0f;
        float f_end = 1000.0f;
        float t_duration = 20.0f;   // Duration of the sweep in seconds
        float t_interval = 5.0f;    // Interval before restarting in seconds

        UserWaveArgs waveArgs(f_start, f_end, t_duration);

        try {
            // Configure the waveform generator once before the loop.
            waveu.configure(waveArgs);
            ESP_LOGI(TAG, "Running Sweep Example");

            while (true) {
                // Start the waveform generation process.
                waveu.start();
                ESP_LOGI(TAG, "Sweep starts at %.01fHz", f_start);

                for (int i = 0; i < (int)t_duration; i++) {
                    vTaskDelay(pdMS_TO_TICKS(1000));  // 1-second delay
                    ESP_LOGI(TAG, "%2ds", i + 1);  // Show elapsed seconds
                }
                ESP_LOGI(TAG, "Sweep finished at %.1fHz. Restarting in %d seconds...", f_end, (int)t_interval);

                // Delay before restarting the sweep.
                vTaskDelay(pdMS_TO_TICKS(t_interval * 1000));

                // Stop waveform generation before restarting.
                waveu.stop();

                // Ensure elapsedTime is reset before starting a new sweep.
                waveu.reset();
            }
        } catch (const tinyalg::waveu::InvalidStateTransitionException& e) {
            ESP_LOGE(TAG, "Exception: %s", e.what());
        }

        // Prevent app_main() from exiting, keeping local variables in memory.
        vTaskDelay(portMAX_DELAY);
    }
}
