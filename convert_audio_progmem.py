#!/usr/bin/env python3
"""
Convert WAV file to Arduino PROGMEM array
Stores audio directly in flash memory for reliable playback without streaming

Based on ChatGPT research and Arduino forum best practices:
- 8kHz, 8-bit, mono audio
- PROGMEM storage in flash
- Timer interrupt playback
"""

import sys
import struct
from pathlib import Path

try:
    from pydub import AudioSegment
except ImportError:
    print("ERROR: pydub not installed. Run: pip install pydub")
    sys.exit(1)

def convert_wav_to_progmem(input_file, output_file=None):
    """Convert WAV to Arduino PROGMEM header file"""
    
    # Auto-generate output filename if not provided
    if output_file is None:
        input_path = Path(input_file)
        output_file = f"{input_path.stem}_data.h"
    
    if not Path(input_file).exists():
        print(f"❌ File not found: {input_file}")
        return False
    
    try:
        print(f"🔄 Converting {input_file} to PROGMEM...")
        
        # Load and convert audio
        audio = AudioSegment.from_file(input_file)
        
        # Convert to mono
        if audio.channels > 1:
            audio = audio.set_channels(1)
            print("✓ Converted to mono")
        
        # Convert to 8kHz
        target_rate = 8000
        if audio.frame_rate != target_rate:
            audio = audio.set_frame_rate(target_rate)
            print(f"✓ Converted to {target_rate}Hz")
        
        # Convert to 8-bit
        if audio.sample_width != 1:
            audio = audio.set_sample_width(1)
            print("✓ Converted to 8-bit")
        
        # Get raw data and convert to unsigned
        raw_data = audio.raw_data
        samples = []
        for i in range(len(raw_data)):
            signed = struct.unpack('b', raw_data[i:i+1])[0]  # -128 to 127
            unsigned = signed + 128  # 0 to 255
            samples.append(unsigned)
        
        duration = len(samples) / target_rate
        size_kb = len(samples) / 1024
        
        print(f"✅ Audio processed: {len(samples)} samples, {duration:.2f}s, {size_kb:.1f}KB")
        
        # Check if it fits in Arduino UNO flash
        if len(samples) > 28000:  # Leave ~4KB for code
            print(f"⚠ Warning: {size_kb:.1f}KB may be too large for Arduino UNO")
            print("  Consider reducing duration or sample rate")
        else:
            print(f"✅ Size OK: {size_kb:.1f}KB fits in Arduino UNO flash")
        
        # Generate Arduino header file
        print(f"📝 Generating {output_file}...")
        
        # Create unique variable names based on filename
        input_path = Path(input_file)
        var_name = input_path.stem.upper().replace('-', '_').replace(' ', '_')
        header_guard = f"{var_name}_DATA_H"
        
        with open(output_file, 'w') as f:
            f.write("/*\n")
            f.write(f" * Audio data converted from {input_file}\n")
            f.write(f" * Duration: {duration:.2f} seconds\n") 
            f.write(f" * Sample rate: {target_rate}Hz\n")
            f.write(f" * Samples: {len(samples)}\n")
            f.write(f" * Size: {size_kb:.1f}KB\n")
            f.write(" * Format: 8-bit unsigned PCM, mono\n")
            f.write(" */\n\n")
            
            f.write(f"#ifndef {header_guard}\n")
            f.write(f"#define {header_guard}\n\n")
            
            f.write("#include <avr/pgmspace.h>\n\n")
            
            f.write(f"const uint16_t {var_name}_SAMPLE_COUNT = {len(samples)};\n")
            f.write(f"const uint16_t {var_name}_SAMPLE_RATE = {target_rate};\n")
            f.write(f"const float {var_name}_DURATION = {duration:.2f};\n\n")
            
            f.write(f"const uint8_t {var_name.lower()}Data[] PROGMEM = {{\n")
            
            # Write samples in rows of 16 for readability
            for i in range(0, len(samples), 16):
                row = samples[i:i+16]
                row_str = ", ".join(f"0x{sample:02X}" for sample in row)
                f.write(f"  {row_str}")
                if i + 16 < len(samples):
                    f.write(",")
                f.write("\n")
            
            f.write("};\n\n")
            f.write(f"#endif // {header_guard}\n")
        
        print(f"✅ Generated {output_file}")
        print("\n🚀 Next steps:")
        print("1. Copy audio_data.h to your Arduino sketch folder")
        print("2. Use the PROGMEM audio player Arduino code")
        print("3. Upload and enjoy reliable audio playback!")
        
        return True
        
    except Exception as e:
        print(f"❌ Conversion failed: {e}")
        return False

def main():
    if len(sys.argv) != 2:
        print("Convert WAV to Arduino PROGMEM")
        print("Usage: python convert_audio_progmem.py <audio_file>")
        print("Example: python convert_audio_progmem.py resample.wav")
        sys.exit(1)
    
    input_file = sys.argv[1]
    
    print("🎵 WAV to Arduino PROGMEM Converter")
    print("Stores audio in flash memory for reliable playback")
    print("=" * 50)
    
    if convert_wav_to_progmem(input_file):
        print("\n🎉 SUCCESS! Audio ready for Arduino PROGMEM playback!")
    else:
        print("\n❌ Conversion failed!")
        sys.exit(1)

if __name__ == "__main__":
    main() 