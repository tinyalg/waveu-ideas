# Waveu Ideas

[Waveu](https://github.com/tinyalg/waveu) is an environment for crafting waveforms with ESP32.  
This repository hosts additional examples to showcase the flexibility and possibilities of Waveu.

## Prerequisites

Before running the examples, ensure you have:

- An **ESP32 development board**.
- ESP-IDF development environment set up (**version 5.4 or later**).
- Familiarity with the ESP-IDF [DAC Continuous Signal Generator Example](https://github.com/espressif/esp-idf/tree/v5.4/examples/peripherals/dac/dac_continuous/signal_generator).
- An **oscilloscope** to visualize the waveform output.

## Usage

1. **Clone the repository**:
   ```bash
   git clone https://github.com/tinyalg/waveu-ideas.git
   ```

2. **Navigate to an example directory**:
   ```bash
   cd waveu-ideas/examples/<example_name>
   ```
   Alternatively, open this directory in VSCode with the ESP-IDF extension for easier navigation and editing.

3. **Run menuconfig**:

   In VSCode, open the ESP-IDF Terminal.
   ```bash
   idf.py menuconfig
   ```
   In `menuconfig`, configure the following
   settings under `[Component config > Waveu Configuration]`:

   - **Select active DAC channels**:
     - **CH0 and CH1**: Output to both channels.
     - **CH0 only**: Output to DAC Channel 0.
     - **CH1 only**: Output to DAC Channel 1.  

4. **Flash the example**:
   ```bash
   idf.py build flash
   ```

5. **Monitor the output**:
   ```bash
   idf.py monitor
   ```

6. **Visualize the waveform**:
   Connect your oscilloscope to the configured DAC output.
