#!/usr/bin/env python3
"""
Tap-Triggered Audio Classifier Pipeline

1. Listens for tap-triggered recordings from STM32
2. Saves WAV file
3. Extracts MFCC features
4. Predicts using pre-trained SVM model

Usage:
  python3 tap_classifier.py -p /dev/cu.usbserial-A5XK3RJT

Requirements:
  pip install pyserial librosa numpy scikit-learn
"""

import argparse
import serial
import struct
import wave
import os
import time
import numpy as np
from datetime import datetime

# For model loading
try:
    import joblib
    JOBLIB_AVAILABLE = True
except ImportError:
    JOBLIB_AVAILABLE = False
    print("Warning: joblib not installed. Run: pip install joblib")

# Optional: for audio features
try:
    import librosa
    LIBROSA_AVAILABLE = True
except ImportError:
    LIBROSA_AVAILABLE = False
    print("Warning: librosa not installed. Run: pip install librosa")

# Protocol markers
START_MARKER = bytes([0xBB, 0x66])
STOP_MARKER = bytes([0xCC, 0x77])
AUDIO_SYNC = bytes([0xAA, 0x55])

# Audio settings
SAMPLE_RATE = 44117
CHANNELS = 1
SAMPLE_WIDTH = 2  # 16-bit


# ============================================================================
#                           FEATURE EXTRACTION
# ============================================================================

def extract_features(audio_path, sr=44100, n_mfcc=13, n_fft=2048, hop_length=512):
    """
    Extract MFCC features from audio file.
    Returns: numpy array of 52 features (13 MFCCs x 4 statistics)
    """
    if not LIBROSA_AVAILABLE:
        raise RuntimeError("librosa is required for feature extraction")

    # Load audio file
    y, sr = librosa.load(audio_path, sr=sr)

    features = []

    # MFCCs (Mel-frequency cepstral coefficients)
    mfccs = librosa.feature.mfcc(y=y, sr=sr, n_mfcc=n_mfcc, n_fft=n_fft,
                                 hop_length=hop_length, n_mels=128)
    mfccs_mean = np.mean(mfccs, axis=1)
    mfccs_std = np.std(mfccs, axis=1)
    mfccs_max = np.max(mfccs, axis=1)
    mfccs_min = np.min(mfccs, axis=1)

    features.extend(mfccs_mean)
    features.extend(mfccs_std)
    features.extend(mfccs_max)
    features.extend(mfccs_min)

    return np.array(features)


def predict_from_file(file_path, model, scaler, label_encoder):
    """
    Extract features and predict class from audio file.
    Returns: (predicted_class, probability_dict)
    """
    # Extract features
    features = extract_features(file_path)
    features_scaled = scaler.transform(features.reshape(1, -1))

    # Make prediction
    prediction = model.predict(features_scaled)[0]
    prediction_proba = model.predict_proba(features_scaled)[0]

    # Get class name
    predicted_class = label_encoder.inverse_transform([prediction])[0]

    # Create probability dictionary using MODEL's class order (not label_encoder)
    probabilities = {}
    for i, class_idx in enumerate(model.classes_):
        class_name = label_encoder.inverse_transform([class_idx])[0]
        probabilities[class_name] = prediction_proba[i]

    return predicted_class, probabilities


# ============================================================================
#                           WAV RECORDING
# ============================================================================

def create_wav_filename(output_dir):
    """Generate unique filename with timestamp"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"quarter_shake_{timestamp}.wav"
    return os.path.join(output_dir, filename)


def save_wav(samples, filename):
    """Save PCM samples to WAV file"""
    with wave.open(filename, 'wb') as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(SAMPLE_WIDTH)
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(samples)

    duration = len(samples) / SAMPLE_WIDTH / SAMPLE_RATE
    return duration


def find_marker(ser, marker, timeout=None):
    """Search for a 2-byte marker in the serial stream."""
    old_timeout = ser.timeout
    if timeout is not None:
        ser.timeout = timeout

    buf = bytes()
    try:
        while True:
            byte = ser.read(1)
            if len(byte) == 0:
                return False

            buf += byte
            if len(buf) >= 2:
                if buf[-2:] == marker:
                    return True
                buf = buf[-1:]
    finally:
        ser.timeout = old_timeout


def receive_audio_frame(ser):
    """Receive one audio frame by scanning for sync markers."""
    prev_byte = None

    while True:
        byte = ser.read(1)
        if len(byte) == 0:
            return None

        if prev_byte is not None:
            two_bytes = prev_byte + byte

            if two_bytes == STOP_MARKER:
                return "STOP"

            if two_bytes == AUDIO_SYNC:
                len_bytes = ser.read(2)
                if len(len_bytes) < 2:
                    return None

                num_samples = struct.unpack('<H', len_bytes)[0]
                num_bytes = num_samples * 2
                pcm_data = ser.read(num_bytes)
                return pcm_data

        prev_byte = byte


# ============================================================================
#                           MAIN PIPELINE
# ============================================================================

def load_model(model_dir):
    """Load pre-trained model, scaler, and label encoder"""
    model_path = os.path.join(model_dir, 'svm_model.pkl')
    scaler_path = os.path.join(model_dir, 'scaler.pkl')
    encoder_path = os.path.join(model_dir, 'label_encoder.pkl')

    print(f"Loading model from {model_dir}...")

    model = joblib.load(model_path)
    scaler = joblib.load(scaler_path)
    label_encoder = joblib.load(encoder_path)

    print(f"  Model loaded: {type(model).__name__}")
    print(f"  Label encoder classes: {list(label_encoder.classes_)}")
    print(f"  Model classes (indices): {list(model.classes_)}")

    return model, scaler, label_encoder


def run_pipeline(port, baudrate, output_dir, model_dir):
    """Main recording and classification loop"""

    # Create output directory
    os.makedirs(output_dir, exist_ok=True)

    # Load ML model
    model, scaler, label_encoder = load_model(model_dir)

    # Open serial port
    print(f"\nOpening {port} at {baudrate} baud...")
    ser = serial.Serial(port, baudrate, timeout=1)
    print("Connected!")

    print()
    print("=" * 60)
    print("       TAP-TRIGGERED AUDIO CLASSIFIER")
    print("=" * 60)
    print(f"  Output directory: {output_dir}")
    print(f"  Sample rate: {SAMPLE_RATE} Hz")
    print(f"  Classes: {list(label_encoder.classes_)}")
    print()
    print("  Waiting for tap to start recording...")
    print("  (Press Ctrl+C to exit)")
    print("=" * 60)
    print()

    recording_count = 0

    try:
        while True:
            # Wait for START marker
            ser.timeout = None
            if not find_marker(ser, START_MARKER):
                continue

            recording_count += 1
            print(f"\n{'='*60}")
            print(f">>> Recording #{recording_count} started!")

            # Collect audio data
            audio_buffer = bytearray()
            frame_count = 0

            ser.timeout = 0.1
            recording_start = time.time()
            max_recording_time = 10

            while True:
                result = receive_audio_frame(ser)

                if result == "STOP":
                    break
                elif result is None:
                    if time.time() - recording_start > max_recording_time:
                        print("  Recording timeout - saving what we have")
                        break
                    continue
                elif len(result) > 0:
                    audio_buffer.extend(result)
                    frame_count += 1

                    if frame_count % 20 == 0:
                        duration = len(audio_buffer) / \
                            SAMPLE_WIDTH / SAMPLE_RATE
                        print(f"  Recording... {duration:.1f}s", end='\r')

            print()
            print(f">>> Recording complete!")

            # Save WAV file
            if len(audio_buffer) > 0:
                filename = create_wav_filename(output_dir)
                duration = save_wav(bytes(audio_buffer), filename)
                print(f"  Saved: {filename}")
                print(f"  Duration: {duration:.2f} seconds")

                # ============ CLASSIFICATION ============
                print()
                print("-" * 40)
                print("  Extracting features...")

                try:
                    predicted_class, probabilities = predict_from_file(
                        filename, model, scaler, label_encoder
                    )

                    print()
                    print("  ╔════════════════════════════════════╗")
                    print(f"  ║  PREDICTION: {predicted_class:^20} ║")
                    print("  ╚════════════════════════════════════╝")
                    print()
                    print("  Confidence scores:")
                    for class_name, prob in sorted(probabilities.items(),
                                                   key=lambda x: x[1], reverse=True):
                        bar = "█" * int(prob * 20)
                        print(f"    {class_name:20s}: {prob:5.1%} {bar}")

                except Exception as e:
                    print(f"  Error during classification: {e}")

                print("-" * 40)
            else:
                print("  Warning: No audio data received!")

            print()
            print("Waiting for next tap...")

    except KeyboardInterrupt:
        print("\n\nExiting...")
    finally:
        ser.close()
        print(f"Total recordings: {recording_count}")


def main():
    parser = argparse.ArgumentParser(
        description='Tap-triggered audio classifier - records, extracts features, and predicts'
    )
    parser.add_argument(
        '-p', '--port',
        required=True,
        help='Serial port (e.g., /dev/cu.usbserial-A5XK3RJT or COM3)'
    )
    parser.add_argument(
        '-b', '--baudrate',
        type=int,
        default=921600,
        help='Baud rate (default: 921600)'
    )
    parser.add_argument(
        '-o', '--output',
        default='./recordings',
        help='Output directory for WAV files (default: ./recordings)'
    )
    parser.add_argument(
        '-m', '--model-dir',
        default='./models',
        help='Directory containing model files (default: ./models)'
    )

    args = parser.parse_args()

    # Check for required libraries
    if not LIBROSA_AVAILABLE:
        print("ERROR: librosa is required. Install with: pip install librosa")
        return

    if not JOBLIB_AVAILABLE:
        print("ERROR: joblib is required. Install with: pip install joblib")
        return

    # Check model files exist
    required_files = ['svm_model.pkl', 'scaler.pkl', 'label_encoder.pkl']
    for f in required_files:
        path = os.path.join(args.model_dir, f)
        if not os.path.exists(path):
            print(f"ERROR: Model file not found: {path}")
            print(f"Make sure all model files are in: {args.model_dir}")
            return

    run_pipeline(args.port, args.baudrate, args.output, args.model_dir)


if __name__ == '__main__':
    main()
