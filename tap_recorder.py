#!/usr/bin/env python3
"""
Tap-Triggered Audio Recorder - Laptop Side

Listens on serial port for START/STOP markers from STM32.
Saves each recording as a separate WAV file.

Protocol:
  START marker: 0xBB 0x66   -> Begin collecting audio
  AUDIO frame:  0xAA 0x55 [len_lo] [len_hi] [PCM data...]
  STOP marker:  0xCC 0x77   -> Save WAV and wait for next

Usage:
  python3 tap_recorder.py -p /dev/cu.usbserial-A5XK3RJT

Requirements:
  pip install pyserial
"""

import argparse
import serial
import struct
import wave
import os
import time
from datetime import datetime

# Protocol markers
START_MARKER = bytes([0xBB, 0x66])
STOP_MARKER = bytes([0xCC, 0x77])
AUDIO_SYNC = bytes([0xAA, 0x55])

# Audio settings
SAMPLE_RATE = 44117
CHANNELS = 1
SAMPLE_WIDTH = 2  # 16-bit


def create_wav_filename(output_dir):
    """Generate unique filename with timestamp"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"recording_{timestamp}.wav"
    return os.path.join(output_dir, filename)


def save_wav(samples, filename):
    """Save PCM samples to WAV file"""
    with wave.open(filename, 'wb') as wf:
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(SAMPLE_WIDTH)
        wf.setframerate(SAMPLE_RATE)
        wf.writeframes(samples)

    duration = len(samples) / SAMPLE_WIDTH / SAMPLE_RATE
    print(f"  Saved: {filename}")
    print(f"  Duration: {duration:.2f} seconds")
    print(f"  Samples: {len(samples) // SAMPLE_WIDTH}")


def find_marker(ser, marker, timeout=None):
    """
    Search for a 2-byte marker in the serial stream.
    Returns True if found, False if timeout.
    """
    old_timeout = ser.timeout
    if timeout is not None:
        ser.timeout = timeout

    buf = bytes()
    try:
        while True:
            byte = ser.read(1)
            if len(byte) == 0:
                # Timeout
                return False

            buf += byte
            if len(buf) >= 2:
                if buf[-2:] == marker:
                    return True
                # Keep only last byte for next comparison
                buf = buf[-1:]
    finally:
        ser.timeout = old_timeout


def receive_audio_frame(ser):
    """
    Receive one audio frame by scanning for sync markers.
    Returns PCM data bytes, "STOP" if stop marker, or None on timeout.
    """
    # Scan byte by byte looking for a marker
    prev_byte = None

    while True:
        byte = ser.read(1)
        if len(byte) == 0:
            return None  # Timeout

        if prev_byte is not None:
            two_bytes = prev_byte + byte

            # Check for STOP marker
            if two_bytes == STOP_MARKER:
                return "STOP"

            # Check for AUDIO sync
            if two_bytes == AUDIO_SYNC:
                # Read length (2 bytes, little endian)
                len_bytes = ser.read(2)
                if len(len_bytes) < 2:
                    return None

                num_samples = struct.unpack('<H', len_bytes)[0]
                num_bytes = num_samples * 2

                # Read PCM data
                pcm_data = ser.read(num_bytes)
                return pcm_data

        prev_byte = byte


def run_recorder(port, baudrate, output_dir):
    """Main recording loop"""

    # Create output directory if needed
    os.makedirs(output_dir, exist_ok=True)

    print(f"Opening {port} at {baudrate} baud...")
    ser = serial.Serial(port, baudrate, timeout=1)
    print("Connected!")
    print()
    print("=" * 50)
    print("  TAP-TRIGGERED AUDIO RECORDER")
    print("=" * 50)
    print(f"  Output directory: {output_dir}")
    print(f"  Sample rate: {SAMPLE_RATE} Hz")
    print()
    print("Waiting for tap to start recording...")
    print("(Press Ctrl+C to exit)")
    print()

    recording_count = 0

    try:
        while True:
            # Wait for START marker
            ser.timeout = None  # Block forever
            if not find_marker(ser, START_MARKER):
                continue

            recording_count += 1
            print(f"\n>>> Recording #{recording_count} started!")

            # Collect audio data
            audio_buffer = bytearray()
            frame_count = 0

            ser.timeout = 0.1  # Short timeout for responsive scanning

            # Safety: max 10 second timeout for entire recording
            recording_start = time.time()
            max_recording_time = 10  # seconds

            while True:
                result = receive_audio_frame(ser)

                if result == "STOP":
                    break
                elif result is None:
                    # Timeout - check if recording took too long
                    if time.time() - recording_start > max_recording_time:
                        print("  Recording timeout - saving what we have")
                        break
                    continue
                elif len(result) > 0:
                    audio_buffer.extend(result)
                    frame_count += 1

                    # Progress indicator
                    if frame_count % 20 == 0:
                        duration = len(audio_buffer) / \
                            SAMPLE_WIDTH / SAMPLE_RATE
                        print(f"  Recording... {duration:.1f}s", end='\r')

            print()  # New line after progress
            print(f">>> Recording #{recording_count} complete!")

            # Save WAV file
            if len(audio_buffer) > 0:
                filename = create_wav_filename(output_dir)
                save_wav(bytes(audio_buffer), filename)
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
        description='Tap-triggered audio recorder - receives audio from STM32 via UART'
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

    args = parser.parse_args()
    run_recorder(args.port, args.baudrate, args.output)


if __name__ == '__main__':
    main()
