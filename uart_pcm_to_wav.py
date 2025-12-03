#!/usr/bin/env python3
"""
UART PCM to WAV Recorder
Receives PCM audio data from STM32 via UART and saves as WAV file.

Usage:
    python uart_pcm_to_wav.py [options]

Examples:
    python uart_pcm_to_wav.py                          # List available ports
    python uart_pcm_to_wav.py -p /dev/cu.usbmodem14203 # Record from specific port
    python uart_pcm_to_wav.py -p /dev/cu.usbmodem14203 -d 10 -o recording.wav
"""

import argparse
import struct
import sys
import time
import wave
from datetime import datetime

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("Error: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)


# ============== Configuration ==============
DEFAULT_BAUD_RATE = 921600
DEFAULT_SAMPLE_RATE = 44100  # Hz - adjust to match your DFSDM config
DEFAULT_CHANNELS = 1         # Mono
DEFAULT_SAMPLE_WIDTH = 2     # 16-bit = 2 bytes

SYNC_BYTE_1 = 0xAA
SYNC_BYTE_2 = 0x55


def list_serial_ports():
    """List all available serial ports."""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial ports found!")
        return []

    print("\nAvailable serial ports:")
    print("-" * 60)
    for port in ports:
        print(f"  {port.device}")
        print(f"    Description: {port.description}")
        if port.manufacturer:
            print(f"    Manufacturer: {port.manufacturer}")
        print()
    return ports


def wait_for_sync(ser):
    """Wait for sync header (0xAA 0x55) and return sample count."""
    state = 0
    while True:
        byte = ser.read(1)
        if not byte:
            continue

        b = byte[0]

        if state == 0:
            if b == SYNC_BYTE_1:
                state = 1
        elif state == 1:
            if b == SYNC_BYTE_2:
                # Found sync! Read 2-byte sample count (little endian)
                count_bytes = ser.read(2)
                if len(count_bytes) == 2:
                    sample_count = struct.unpack('<H', count_bytes)[0]
                    return sample_count
            elif b == SYNC_BYTE_1:
                state = 1  # Stay in state 1
            else:
                state = 0


def record_audio(port, baud_rate, duration, sample_rate, output_file):
    """Record audio from UART and save as WAV."""

    print(f"\nConnecting to {port} at {baud_rate} baud...")

    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud_rate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return False

    # Clear any pending data
    ser.reset_input_buffer()

    print(f"Recording for {duration} seconds...")
    print(f"Sample rate: {sample_rate} Hz")
    print(f"Output file: {output_file}")
    print("\nPress Ctrl+C to stop early\n")

    all_samples = bytearray()
    start_time = time.time()
    frames_received = 0
    total_samples = 0

    try:
        while (time.time() - start_time) < duration:
            # Wait for sync and get sample count
            sample_count = wait_for_sync(ser)

            if sample_count == 0 or sample_count > 4096:
                # Sanity check - skip invalid counts
                continue

            # Read PCM data (2 bytes per sample)
            bytes_to_read = sample_count * 2
            pcm_data = ser.read(bytes_to_read)

            if len(pcm_data) == bytes_to_read:
                all_samples.extend(pcm_data)
                frames_received += 1
                total_samples += sample_count

                # Progress indicator
                elapsed = time.time() - start_time
                print(f"\r  Frames: {frames_received} | "
                      f"Samples: {total_samples:,} | "
                      f"Time: {elapsed:.1f}s / {duration}s | "
                      f"Size: {len(all_samples)/1024:.1f} KB", end='')

    except KeyboardInterrupt:
        print("\n\nRecording stopped by user.")

    finally:
        ser.close()

    # Save as WAV
    if len(all_samples) > 0:
        print(f"\n\nSaving to {output_file}...")

        with wave.open(output_file, 'wb') as wav_file:
            wav_file.setnchannels(DEFAULT_CHANNELS)
            wav_file.setsampwidth(DEFAULT_SAMPLE_WIDTH)
            wav_file.setframerate(sample_rate)
            wav_file.writeframes(bytes(all_samples))

        duration_recorded = total_samples / sample_rate
        file_size = len(all_samples) / 1024

        print(f"\nRecording saved!")
        print(f"  Duration: {duration_recorded:.2f} seconds")
        print(f"  Samples:  {total_samples:,}")
        print(f"  Size:     {file_size:.1f} KB")
        return True
    else:
        print("\nNo audio data received!")
        return False


def main():
    parser = argparse.ArgumentParser(
        description='Record PCM audio from STM32 UART and save as WAV file.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s                                    # List available serial ports
  %(prog)s -p /dev/cu.usbmodem14203           # Record with defaults
  %(prog)s -p /dev/cu.usbmodem14203 -d 10     # Record for 10 seconds
  %(prog)s -p COM3 -b 921600 -d 30            # Windows, faster baud, 30 sec
        """
    )

    parser.add_argument('-p', '--port',
                        help='Serial port (e.g., /dev/cu.usbmodem14203 or COM3)')
    parser.add_argument('-b', '--baud', type=int, default=DEFAULT_BAUD_RATE,
                        help=f'Baud rate (default: {DEFAULT_BAUD_RATE})')
    parser.add_argument('-d', '--duration', type=float, default=5.0,
                        help='Recording duration in seconds (default: 5)')
    parser.add_argument('-s', '--samplerate', type=int, default=DEFAULT_SAMPLE_RATE,
                        help=f'Sample rate in Hz (default: {DEFAULT_SAMPLE_RATE})')
    parser.add_argument('-o', '--output',
                        help='Output WAV filename (default: recording_TIMESTAMP.wav)')

    args = parser.parse_args()

    # If no port specified, list available ports
    if not args.port:
        list_serial_ports()
        print("Tip: Run with -p <port> to start recording")
        print("     Example: python uart_pcm_to_wav.py -p /dev/cu.usbmodem14203")
        return

    # Generate default filename if not specified
    if not args.output:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        args.output = f"recording_{timestamp}.wav"

    # Start recording
    success = record_audio(
        port=args.port,
        baud_rate=args.baud,
        duration=args.duration,
        sample_rate=args.samplerate,
        output_file=args.output
    )

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
