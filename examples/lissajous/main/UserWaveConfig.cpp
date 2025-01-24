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
lut_type_t lutX[LUT_1024];

/// @brief LUT : Select an LUT size from the predefined LUTSize's.
lut_type_t lutY[LUT_1024];

// Derived class for user wave arguments
class UserWaveArgs : public tinyalg::waveu::WaveConfigArgs {
public:
    float A;
    float B;
    float a;
    float b;
    float offset;
    float delta;

    UserWaveArgs(float A, float B, float a, float b, float offset, float delta)
        : A(A), B(B), a(a), b(b), offset(offset), delta(delta) {}
};

class UserWaveConfig : public tinyalg::waveu::WaveConfigInterface {
protected:
    virtual lut_type_t fill_function_sine(LUTSize size, size_t index, float amplitude, float offset, float delta = 0) {
        size_t table_size = static_cast<size_t>(size);
        return static_cast<lut_type_t>(
            amplitude * sin(index * 2 * M_PI / (table_size - 1) + delta) + offset + 0.5
        );
    }

    virtual void initializePhaseGenerator(uint32_t sampleRate) {
        if (_phaseGeneratorX == nullptr) {
            _phaseGeneratorX = new PhaseGenerator(sampleRate);
        }
        if (_phaseGeneratorY == nullptr) {
            _phaseGeneratorY = new PhaseGenerator(sampleRate);
        }
    }

    virtual void setPhaseGeneratorFrequency(float frequency1, float frequency2) {
        _phaseGeneratorX->setFrequency(frequency1);
        _phaseGeneratorY->setFrequency(frequency2);
        ESP_LOGI(TAG, "Frequency set to X: %.02fHz, Y: %.02fHz",
            _phaseGeneratorX->getFrequency(), _phaseGeneratorY->getFrequency());
    }

    virtual void populateLUTs(float amplitude1, float amplitude2, float offset1, float offset2, float delta) {
        size_t table_size = static_cast<size_t>(_lutSize);
        for (size_t i = 0; i < table_size; ++i) {
            lutX[i] = fill_function_sine(_lutSize, i, amplitude1, offset1);
            lutY[i] = fill_function_sine(_lutSize, i, amplitude2, offset2, delta);
        }
    }

public:
    ~UserWaveConfig() override {
        delete _phaseGeneratorX; // Clean up
        delete _phaseGeneratorY; // Clean up
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
        setPhaseGeneratorFrequency(waveArgs.a, waveArgs.b);

        // Validates and adjusts amplitude and offset for the 8-bit DAC.
        auto [adjustedA, adjustedOffsetA] = LUTHelper::adjustAmplitudeAndOffset(waveArgs.A, waveArgs.offset);
        auto [adjustedB, adjustedOffsetB] = LUTHelper::adjustAmplitudeAndOffset(waveArgs.B, waveArgs.offset);

        // Fills the values to the LUT using the supplied function.
        populateLUTs(adjustedA, adjustedB, adjustedOffsetA, adjustedOffsetB, waveArgs.delta);
    }

    void prepareCycle(double elapsedTime) override {
        _phaseGeneratorX->reset(elapsedTime);
        _phaseGeneratorY->reset(elapsedTime);
    }

    uint8_t nextSample() override {
        // Step 1: Advance the phase to the next position.
        _phaseGeneratorX->updatePhase();

        // Step 2: Retrieve the updated phase value.
        uint32_t currentPhase = _phaseGeneratorX->getPhase();

        // Step 3: Map the phase value to an appropriate index in the lookup table.
        int lutIndex = _getIndex(currentPhase, PhaseGenerator::N_BITS);

        // Step 4: Fetch the corresponding voltage value (0-255) from the lookup table using the macro.
        lut_type_t digi_val = GET_LUT_VALUE(lutX, lutIndex);

        // Return the voltage value for the sample.
        return (uint8_t)digi_val;
    }

    uint8_t nextSampleB() override {
        // Step 1: Advance the phase to the next position.
        _phaseGeneratorY->updatePhase();

        // Step 2: Retrieve the updated phase value.
        uint32_t currentPhase = _phaseGeneratorY->getPhase();

        // Step 3: Map the phase value to an appropriate index in the lookup table.
        int lutIndex = _getIndex(currentPhase, PhaseGenerator::N_BITS);

        // Step 4: Fetch the corresponding voltage value (0-255) from the lookup table using the macro.
        lut_type_t digi_val = GET_LUT_VALUE(lutY, lutIndex);
        
        // Return the voltage value for the sample.
        return (uint8_t)digi_val;
    }

    void reset() override {
    }

private:
    /// @brief Pointer to PhaseGenerator
    PhaseGenerator* _phaseGeneratorX = nullptr;

    /// @brief Pointer to PhaseGenerator
    PhaseGenerator* _phaseGeneratorY = nullptr;

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

        float A = 127.5f;
        float B = 127.5f;
        float offset = 127.5f;

        // Feel free to modify 'a', 'b', or 'delta' to explore other waveforms.
        float a = 0.0f;
        float b = 0.0f;
        float delta = 0.0f;

        // Select from predefined examples:
        // 2: Visualizing Phase Differences
        // 3: Visualizing Frequency Ratios
        // 4: Combined Impact of Phase and Frequency
        // Any other value defaults to a circle.
        constexpr int select_example = 2;

        switch (select_example) {
        case 2:
            a = b = 200.0f; delta = M_PI / 4.0f;
            break;
        case 3:
            a = 200.0f; b = 100.0f; delta = 0.0f;
            break;
        case 4:
            a = 300.0f; b = 200.0f; delta = M_PI / 4.0f;
            break;
        default:
            a = b = 200.0f; delta = M_PI / 2;
            break;
        }
        UserWaveArgs waveArgs(A, B, a, b, offset, delta);

        try {
            // Configure the waveform generator with the specified arguments.
            waveu.configure(waveArgs);

            // Start the waveform generation process.
            waveu.start();
            ESP_LOGI(TAG, "Running Example %d", select_example);

        } catch (const tinyalg::waveu::InvalidStateTransitionException& e) {
            ESP_LOGE(TAG, "Exception: %s\n", e.what());
        }

        // Prevent app_main() from exiting, keeping local variables in memory.
        vTaskDelay(portMAX_DELAY);
    }
}
