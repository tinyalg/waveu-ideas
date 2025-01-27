#include <cstdint>
#include <cmath>     // For sin, M_PI
#include <utility>   // For std::pair
#include <algorithm> // For std::clamp
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
lut_type_t lutC[LUT_1024];
lut_type_t lutM[LUT_1024];

// Derived class for user wave arguments
class UserWaveArgs : public tinyalg::waveu::WaveConfigArgs {
public:
    float A_c;
    float A_m;
    float f_c;
    float f_m;

    UserWaveArgs(float A_c, float A_m, float f_c, float f_m)
        : A_c(A_c), A_m(A_m), f_c(f_c), f_m(f_m) {}
};

class UserWaveConfig : public tinyalg::waveu::WaveConfigInterface {
protected:
    virtual std::pair<float, float> adjustAmplitude(float amplitude1, float amplitude2) {
        constexpr float MAX_AMPLITUDE = 1.0f; // Ensures the waveform does not exceed the DAC output range.
        constexpr float MIN_AMPLITUDE = 0.0f;

        // Step 1: Clamp amplitudes to valid range
        amplitude1 = std::clamp(amplitude1, MIN_AMPLITUDE, MAX_AMPLITUDE);
        amplitude2 = std::clamp(amplitude2, MIN_AMPLITUDE, MAX_AMPLITUDE);

        // Step 2: Normalize if the sum exceeds MAX_AMPLITUDE
        float sum = amplitude1 + amplitude2;
        if (sum > MAX_AMPLITUDE) {
            amplitude1 = (amplitude1 / sum) * MAX_AMPLITUDE;
            amplitude2 = (amplitude2 / sum) * MAX_AMPLITUDE;
        }

        return std::make_pair(amplitude1, amplitude2);
    }

    virtual lut_type_t fill_function_sine(LUTSize size, size_t index, float amplitude) {
            size_t table_size = static_cast<size_t>(size);
            const int max_amplitude = 2032; // Chosen as 2^4 * output amplitude 127 to maintain accuracy
                                            // during intermediate calculations.
            return static_cast<tinyalg::waveu::lut_type_t>(
                max_amplitude * amplitude * sin(index * 2 * M_PI / (table_size - 1)) + 0.5
            );
    }

    virtual lut_type_t fill_function_cosine(LUTSize size, size_t index, float amplitude) {
            size_t table_size = static_cast<size_t>(size);
            const int max_amplitude = 1016; // == 1/2 * 2^4 * output amplitude 127
            return static_cast<tinyalg::waveu::lut_type_t>(
                max_amplitude * amplitude * cos(index * 2 * M_PI / (table_size - 1)) + 0.5
            );
    }

    virtual void initializePhaseGenerator(uint32_t sampleRate) {
        // Initialize separate PhaseGenerators for the carrier, lower sideband, and upper sideband.
        // This ensures each component has independent phase control.
        if (_phaseGeneratorC == nullptr) {
            _phaseGeneratorC = new PhaseGenerator(sampleRate); // Initialize carrier phase generator
        }
        if (_phaseGeneratorL == nullptr) {
            _phaseGeneratorL = new PhaseGenerator(sampleRate); // Initialize lower sideband phase generator
        }
        if (_phaseGeneratorU == nullptr) {
            _phaseGeneratorU = new PhaseGenerator(sampleRate); // Initialize upper sideband phase generator
        }
    }

    virtual void setPhaseGeneratorFrequency(float f_c, float f_m) {
        _phaseGeneratorC->setFrequency(f_c);
        _phaseGeneratorL->setFrequency(f_c - f_m);
        _phaseGeneratorU->setFrequency(f_c + f_m);

        ESP_LOGI(TAG, "Frequency set to Carrier: %.02fHz, LSB: %.02fHz, USB: %.02fHz",
            _phaseGeneratorC->getFrequency(), _phaseGeneratorL->getFrequency(), _phaseGeneratorU->getFrequency());
    }

    virtual void populateLUTs(float A_c, float A_m) {
        // Separate LUTs are chosen for the carrier and sidebands to avoid real-time division by 2.
        // This ensures computational efficiency during waveform generation.
        size_t table_size = static_cast<size_t>(_lutSize);
        for (size_t i = 0; i < table_size; ++i) {
            lutC[i] = fill_function_sine(_lutSize, i, A_c);
        }
        for (size_t i = 0; i < table_size; ++i) {
            lutM[i] = fill_function_cosine(_lutSize, i, A_m);
        }
    }

public:
    ~UserWaveConfig() override {
        delete _phaseGeneratorC; // Clean up
        delete _phaseGeneratorL; // Clean up
        delete _phaseGeneratorU; // Clean up
    }

    void initialize(uint32_t sampleRate) override {
        // Initializes the PhaseGenerator with the specified sample rate.
        initializePhaseGenerator(sampleRate);

        // Retrieves a function to calculate the LUT index from a phase.
        _getIndex = LUTHelper::getIndexFunction(_lutSize);
    }

    void configure(const tinyalg::waveu::WaveConfigArgs& args) override {
        const auto& waveArgs = dynamic_cast<const UserWaveArgs&>(args); // Safe because caller ensures type

        // Sets the desired frequency and calculates the phase increment.
        setPhaseGeneratorFrequency(waveArgs.f_c, waveArgs.f_m);

        // Validates and adjusts amplitude in [0.0, 1.0]
        auto [adjustedA_c, adjustedA_m] = adjustAmplitude(waveArgs.A_c, waveArgs.A_m);

        // Fills the values to the LUT using the supplied function.
        populateLUTs(adjustedA_c, adjustedA_m);
    }

    void prepareCycle(double elapsedTime) override {
        _phaseGeneratorC->reset(elapsedTime);
        _phaseGeneratorL->reset(elapsedTime);
        _phaseGeneratorU->reset(elapsedTime);
    }

    uint8_t nextSample() override {
        // Step 1: Advance the phase to the next position.
        _phaseGeneratorC->updatePhase();
        _phaseGeneratorL->updatePhase();
        _phaseGeneratorU->updatePhase();

        // Step 2: Retrieve the updated phase value.
        uint32_t currentPhaseC = _phaseGeneratorC->getPhase();
        uint32_t currentPhaseL = _phaseGeneratorL->getPhase();
        uint32_t currentPhaseU = _phaseGeneratorU->getPhase();

        // Step 3: Map the phase value to an appropriate index in the lookup table.
        int lutIndexC = _getIndex(currentPhaseC, PhaseGenerator::N_BITS);
        int lutIndexL = _getIndex(currentPhaseL, PhaseGenerator::N_BITS);
        int lutIndexU = _getIndex(currentPhaseU, PhaseGenerator::N_BITS);

        // Step 4: Fetch the corresponding voltage value (0-255) from the lookup table using the macro.
        lut_type_t carrier = GET_LUT_VALUE(lutC, lutIndexC);
        lut_type_t lower_sideband = GET_LUT_VALUE(lutM, lutIndexL);
        lut_type_t upper_sideband = -GET_LUT_VALUE(lutM, lutIndexU);

        // AM signal
        int am_signal = carrier + lower_sideband + upper_sideband;

        // Scale for LUT resolution (division by 16)
        int digi_val = (am_signal >> 4) + 127; // Offset to DAC range

        // Return the voltage value for the sample.
        return (uint8_t)digi_val;
    }

    void reset() override {
    }

private:
    /// @brief Pointer to PhaseGenerator for carrier component
    PhaseGenerator* _phaseGeneratorC = nullptr;

    /// @brief Pointer to PhaseGenerator for lower sideband component
    PhaseGenerator* _phaseGeneratorL = nullptr;
    
    /// @brief Pointer to PhaseGenerator for upper sideband component
    PhaseGenerator* _phaseGeneratorU = nullptr;

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

        float A_c = 0.6f;
        float A_m = 0.4f;
        float f_c = 500.0f;
        float f_m = 100.0f;

        UserWaveArgs waveArgs(A_c, A_m, f_c, f_m);

        try {
            // Configure the waveform generator with the specified arguments.
            waveu.configure(waveArgs);

            // Start the waveform generation process.
            waveu.start();
            ESP_LOGI(TAG, "Running Amplitude Modulation Example");

        } catch (const tinyalg::waveu::InvalidStateTransitionException& e) {
            ESP_LOGE(TAG, "Exception: %s\n", e.what());
        }

        // Prevent app_main() from exiting, keeping local variables in memory.
        vTaskDelay(portMAX_DELAY);
    }
}
