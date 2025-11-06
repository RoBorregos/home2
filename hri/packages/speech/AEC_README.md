# Acoustic Echo Cancellation (AEC) Implementation

This directory contains an STFT-based Acoustic Echo Cancellation system integrated into the HRI speech pipeline.

## Overview

The AEC system cancels acoustic echo (robot's voice captured by the microphone) to improve speech recognition quality. It uses Short-Time Fourier Transform (STFT) with a Wiener filter approach in the frequency domain.

## Architecture

```
┌─────────────────┐
│  audio_capturer │ ─────► /rawAudioChunk ─────┐
└─────────────────┘                             │
                                                 ▼
┌─────────────────┐                        ┌──────────┐
│      say        │ ─────► /robot_audio ──►│ aec_node │─────► /aec_audio_output
└─────────────────┘        _output         └──────────┘              │
                                                                      ▼
                                                            ┌──────────────────┐
                                                            │ hear_streaming   │
                                                            └──────────────────┘
```

### Components

1. **stft_aec.py**: Core AEC algorithm

   - `STFTEchoCanceller`: Main AEC class with STFT processing
   - Delay estimation via cross-correlation
   - Wiener filtering in frequency domain
   - Supports both batch and streaming processing

2. **aec_node.py**: ROS2 node for real-time AEC

   - Subscribes to microphone input (`/rawAudioChunk`)
   - Subscribes to robot audio output (`/robot_audio_output`)
   - Publishes echo-cancelled audio (`/aec_audio_output`)
   - Automatic pass-through when robot is not speaking

3. **Modified nodes**:
   - **say.py**: Now publishes audio data to `/robot_audio_output` for AEC reference
   - **hear_streaming.py**: Now subscribes to `/aec_audio_output` instead of raw audio

## Configuration

Edit `config/aec.yaml`:

```yaml
aec_node:
  ros__parameters:
    enabled: true # Enable/disable AEC
    frame_size: 1024 # STFT frame size (samples)
    hop_size: 256 # STFT hop size (samples)
    beta: 0.9 # PSD smoothing [0..1]
    sample_rate: 16000 # Sample rate (Hz)
    max_delay_ms: 200.0 # Max delay search (ms)
    enable_normalization: true
```

### Parameters

- **frame_size**: Larger = better frequency resolution, more latency
- **hop_size**: Smaller = better time resolution, more computation
- **beta**: Higher = smoother filter, slower adaptation. Lower = faster adaptation, more noise
- **max_delay_ms**: Maximum acoustic delay to compensate (increase if robot/mic are far apart)

## Usage

### Automatic (via launch file)

The AEC node is automatically started with the HRI launch:

```bash
ros2 launch speech devices_launch.py
```

### Manual testing

Test the AEC with audio files:

```bash
cd hri/packages/speech/scripts
python3 test_aec_offline.py \
    --robot robot_audio.wav \
    --mic microphone_capture.wav \
    --output_dir ./aec_test_results
```

### ROS2 topics

Monitor AEC status:

```bash
ros2 topic echo /aec_status
```

Check if AEC is processing:

```bash
ros2 node info /aec
```

## How It Works

### 1. Signal Alignment

The AEC first estimates the delay between the robot audio output and its echo in the microphone using cross-correlation. This is critical for proper echo cancellation.

### 2. STFT Processing

Audio is processed in overlapping frames using the Short-Time Fourier Transform:

- Frame the signals with a Hanning window
- Transform to frequency domain
- Process each frequency bin independently

### 3. Wiener Filtering

For each frequency bin:

- Estimate auto-PSD of reference: `Pxx = |X|²`
- Estimate cross-PSD: `Pxd = X* · D`
- Calculate filter: `H = Pxd / (Pxx + ε)`
- Estimate echo: `Y = H · X`
- Calculate residual: `E = D - Y`

These PSDs are smoothed over time using exponential averaging with factor `beta`.

### 4. Overlap-Add

Inverse STFT with overlap-add synthesis to reconstruct the time-domain signal.

## Performance Tips

1. **Acoustic Setup**

   - Keep microphone and speaker separated
   - Reduce room reverberation
   - Use directional microphone when possible

2. **Parameter Tuning**

   - Increase `beta` for stable environments
   - Decrease `beta` for quickly changing acoustics
   - Adjust `max_delay_ms` based on physical setup

3. **Latency Optimization**
   - Reduce `frame_size` and `hop_size` for lower latency
   - Trade-off: smaller frames = less frequency resolution

## Testing

The system includes offline testing capabilities:

```bash
# Record test audio
# 1. Record robot speaking (reference)
# 2. Record microphone while robot speaks (with echo)

# Run AEC test
python3 test_aec_offline.py \
    --robot reference.wav \
    --mic mic_with_echo.wav \
    --frame 1024 \
    --hop 256 \
    --beta 0.9 \
    --prefix test1

# Compare:
# - test1_mic_aligned.wav (original mic)
# - test1_echo_cancelled.wav (processed)
# - test1_robot_estimate.wav (estimated echo)
```

## Troubleshooting

### Poor echo cancellation

- Check delay estimation (should be printed in logs)
- Verify audio synchronization
- Increase `max_delay_ms` if delay > current setting
- Tune `beta` parameter

### Audio distortion

- Reduce `beta` for faster adaptation
- Check for clipping in input signals
- Verify sample rate matches (16kHz)

### High latency

- Reduce `frame_size` and `hop_size`
- Check buffer sizes in config

### No output

- Verify `/robot_audio_output` is being published
- Check AEC node is running: `ros2 node list`
- Check topic connections: `ros2 topic info /aec_audio_output`

## Technical References

The implementation is based on:

- Wiener filtering in STFT domain
- Exponential PSD smoothing
- Cross-correlation delay estimation

For more details on adaptive filtering and AEC:

- Haykin, S. "Adaptive Filter Theory"
- Loizou, P. "Speech Enhancement: Theory and Practice"

## Future Improvements

Possible enhancements:

- [ ] Adaptive step size for faster convergence
- [ ] Double-talk detection
- [ ] Nonlinear echo suppression
- [ ] Frequency-dependent smoothing
- [ ] GPU acceleration for real-time processing
- [ ] Multi-channel AEC for arrays
