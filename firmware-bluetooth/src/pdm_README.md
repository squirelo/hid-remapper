# PDM Microphone Module

This module provides PDM (Pulse Density Modulation) microphone support for the HID Remapper Bluetooth firmware, following the same pattern as the IMU module.

## Features

- **Audio Level Detection**: RMS (Root Mean Square) calculation for audio intensity
- **Frequency Analysis**: Basic frequency estimation using zero-crossing detection
- **Volume Monitoring**: Peak detection for volume levels
- **Filtering**: IIR filters for smooth audio processing
- **HID Integration**: Reports audio data as HID sensor reports

## Configuration

The PDM module can be enabled/disabled via the `pdm_enabled` configuration variable.

## Hardware Requirements

- PDM microphone connected to PDM0 interface
- Pins configured in devicetree overlay:
  - PDM_DIN: Pin 0.26 (data input)
  - PDM_CLK: Pin 0.25 (clock signal)

## Report Structure

The PDM module generates HID reports with the following structure:

```c
typedef struct {
    int16_t audio_level;  // RMS audio level (-32768 to +32767)
    uint16_t frequency;    // Estimated frequency (0 to 65535 Hz)
    uint8_t volume;        // Peak volume (0 to 255)
} pdm_report_t;
```

## Usage

1. **Initialization**: Call `pdm_init()` after the mapping system is ready
2. **Recalibration**: Call `pdm_recalibrate()` to reset filters
3. **Cleanup**: Call `pdm_stop()` to stop PDM operation

## Integration

The module is automatically integrated into the main firmware and will:
- Initialize when `pdm_enabled` is true and PDM hardware is available
- Generate HID reports on virtual interface 0x2000
- Process audio data at 100Hz (10ms intervals)
- Apply configurable filters and thresholds

## Configuration Parameters

- `alpha`: IIR filter coefficient for smoothing (default: 0.9)
- `beta`: IIR filter coefficient for frequency (default: 0.1)
- `threshold`: Minimum audio level threshold (default: 0.01)
- `gain`: Audio gain multiplier (default: 1.0)

## Error Handling

The module includes robust error handling:
- Automatic retry with exponential backoff on errors
- Graceful degradation when hardware is unavailable
- LED activity indication during operation 