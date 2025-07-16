# Solar Shrine Custom Arduino Shield Design

## 🎯 **Project Overview**

Custom Arduino shield designed to consolidate all Solar Shrine components into a compact, waterproof-case-friendly layout. This shield stacks the **WWZMDiB XH-M543 High Power Digital Amplifier** directly onto the Arduino and provides clean connections for all external sensors and components.

---

## 📐 **Shield Specifications**

### **Board Dimensions**
- **Shield PCB**: 68.6mm x 53.4mm (Arduino Uno standard)
- **Total Height**: ~32mm (Arduino + Shield + Amplifier)
- **Amplifier Module**: 92mm x 68mm x 16mm (WWZMDiB XH-M543)
- **Recommended Enclosure**: Hammond 1590WYF (92x92x42mm) - **Perfect fit!**

### **Key Features**
- ✅ **Direct amplifier stacking** - No loose wires
- ✅ **Waterproof connectors** - JST-XH series for field use
- ✅ **Perfboard section** - For custom circuits and modifications
- ✅ **Status indicators** - Power and activity LEDs
- ✅ **Multiple power options** - Barrel jack + terminal blocks
- ✅ **Professional audio output** - 3.5mm jack + screw terminals

---

## 🔧 **Component Layout**

### **Shield Component Placement**

```
    ┌─────────────────────────────────────────────────────────────────┐
    │  SOLAR SHRINE CUSTOM SHIELD - TOP VIEW (68.6mm x 53.4mm)       │
    │                                                                 │
    │  ┌─────────────────────────────────────┐  ┌─────────────────┐   │
    │  │     WWZMDiB XH-M543 AMPLIFIER      │  │   PERFBOARD     │   │
    │  │        92mm x 68mm x 16mm           │  │   SECTION       │   │
    │  │                                     │  │  20x15 holes    │   │
    │  │  ┌──┐ TPA3116D2 ┌──┐                │  │  0.1" spacing   │   │
    │  │  │IN│           │OUT               │  │                 │   │
    │  │  └──┘           └──┘                │  └─────────────────┘   │
    │  └─────────────────────────────────────┘                      │
    │                                                                 │
    │  ┌──────┐ ┌──────┐ ┌──────┐  ┌────────┐  ┌─────┐ ┌─────┐      │
    │  │SENSOR│ │SENSOR│ │ LED  │  │ POWER  │  │AUDIO│ │STATUS     │
    │  │  1   │ │  2   │ │STRIP │  │INPUT   │  │ OUT │ │LEDS │      │
    │  │JST-XH│ │JST-XH│ │JST-XH│  │BARREL  │  │3.5mm│ │PWR  │      │
    │  │4-pin │ │4-pin │ │3-pin │  │ JACK   │  │JACK │ │ACT  │      │
    │  └──────┘ └──────┘ └──────┘  └────────┘  └─────┘ └─────┘      │
    │                                                                 │
    │  ┌─────────────────────────────────────────────────────────┐   │
    │  │           ARDUINO UNO HEADER PINS                      │   │
    │  │  ┌────────────────┐         ┌────────────────────────┐  │   │
    │  │  │ DIGITAL 0-7    │         │ DIGITAL 8-13 + POWER  │  │   │
    │  │  └────────────────┘         └────────────────────────┘  │   │
    │  │  ┌────────────────┐         ┌────────────────────────┐  │   │
    │  │  │ ANALOG A0-A5   │         │ POWER + RESET          │  │   │
    │  │  └────────────────┘         └────────────────────────┘  │   │
    │  └─────────────────────────────────────────────────────────┘   │
    └─────────────────────────────────────────────────────────────────┘
```

---

## 🔌 **Connector Specifications**

### **1. Sensor Connectors (2x JST-XH 4-pin)**
- **Purpose**: HC-SR04 ultrasonic sensors with extended cables
- **Pinout**: VCC, GND, Trig, Echo
- **Wire Gauge**: 22-24 AWG recommended
- **Cable Length**: Up to 3 meters (shielded cable recommended)

| Pin | Sensor 1 (Left) | Sensor 2 (Right) | Arduino Mega Pin |
|-----|------------------|-------------------|------------------|
| 1   | VCC (5V)         | VCC (5V)          | 5V               |
| 2   | GND              | GND               | GND              |
| 3   | Trig             | Trig              | Pin 10           |
| 4   | Echo             | Echo              | Pin 11           |
| 1   | VCC (5V)         | VCC (5V)          | 5V               |
| 2   | GND              | GND               | GND              |
| 3   | Trig             | Trig              | Pin 5            |
| 4   | Echo             | Echo              | Pin 6            |

### **2. LED Strip Connector (JST-XH 3-pin)**
- **Purpose**: WS2812B/WS2815 LED strip connection
- **Pinout**: VCC, GND, Data
- **Power**: Up to 5A for WS2812B (5V) or 3A for WS2815 (12V)

| Pin | Function | Arduino Pin | Notes |
|-----|----------|-------------|-------|
| 1   | VCC      | External 5V/12V | From power input |
| 2   | GND      | GND         | Common ground |
| 3   | Data     | Pin 3       | Signal line |

### **3. Audio Output**
- **3.5mm Stereo Jack**: For headphones/external speakers
- **Screw Terminals**: For permanent speaker connections
- **Source**: WWZMDiB XH-M543 amplifier output

### **4. Power Input**
- **Barrel Jack**: 12V DC input (2.1mm center positive)
- **Terminal Block**: Alternative 12V input for permanent installation
- **Capacity**: Up to 5A for full system operation

---

## 🛠️ **Perfboard Section Design**

### **20x15 Hole Array (0.1" spacing)**
```
    A B C D E F G H I J K L M N O P Q R S T
 1  ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ●
 2  ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ●
 3  ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ●
 4  ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ●
 5  ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ●
 6  ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ●
 7  ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ●
 8  ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ●
 9  ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ●
10  ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ●
11  ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ●
12  ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ●
13  ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ●
14  ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ●
15  ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ● ●
```

### **Perfboard Usage Ideas**
- **Volume control potentiometer** (10kΩ linear)
- **Additional status LEDs** for debugging
- **Pull-up resistors** for I2C expansion
- **Voltage dividers** for analog sensors
- **Decoupling capacitors** for power conditioning
- **Jumper wire connections** for configuration changes

---

## 📦 **Recommended Waterproof Enclosure**

### **Hammond 1590WYF** - Perfect Match!
- **External Dimensions**: 92mm x 92mm x 42mm
- **Internal Clearance**: ~84mm x 84mm x 36mm
- **IP Rating**: IP65 (waterproof)
- **Material**: Die-cast aluminum
- **Weight**: 0.64 lbs (290g)
- **Cost**: ~$15-20

### **Why This Enclosure is Perfect:**
✅ **Exact fit for amplifier** (92mm x 68mm)
✅ **Sufficient height** for stacked components (42mm total)
✅ **Professional appearance** with natural aluminum finish
✅ **Waterproof rating** suitable for outdoor installation
✅ **Easy machining** for connector holes
✅ **Good heat dissipation** for amplifier

### **Alternative Enclosures:**
- **Hammond 1590WGF**: 100mm x 50mm x 25mm (too narrow)
- **Hammond 1590WH**: 53mm x 38mm x 31mm (too small)
- **Larger option**: Hammond 1590WZ (120mm x 120mm x 59mm) for future expansion

---

## 🔧 **Assembly Instructions**

### **Step 1: Shield PCB Assembly**
1. **Solder Arduino headers** - Use stackable headers for future expansion
2. **Install JST-XH connectors** - Position for easy cable access
3. **Mount barrel jack and terminal blocks** - Secure with nuts
4. **Add status LEDs and resistors** - 330Ω current limiting resistors
5. **Wire perfboard section** - Add any custom components needed

### **Step 2: Amplifier Integration**
1. **Mount amplifier to shield** - Use standoffs or direct mounting
2. **Connect audio input** - From Arduino pin 11 to amplifier input
3. **Connect power** - 12V from shield power input to amplifier
4. **Connect audio output** - From amplifier to 3.5mm jack and terminals

### **Step 3: Enclosure Preparation**
1. **Drill connector holes**:
   - 2x 12mm holes for JST-XH sensor connectors
   - 1x 8mm hole for JST-XH LED connector  
   - 1x 8mm hole for 3.5mm audio jack
   - 1x 12mm hole for barrel jack
   - 2x 3mm holes for status LEDs
2. **Install cable glands** - For waterproof cable entry
3. **Add ventilation** - Small vent for condensation (if needed)

### **Step 4: Final Assembly**
1. **Install shield on Arduino** - Ensure proper pin alignment
2. **Mount assembly in enclosure** - Use standoffs for vibration resistance
3. **Connect external cables** - Sensors, LED strip, power
4. **Test all functions** - Upload and run main theremin code
5. **Seal enclosure** - Apply gasket and torque screws evenly

---

## ⚡ **Power Distribution**

### **Power Input Options**
- **Primary**: 12V barrel jack (center positive)
- **Secondary**: 12V terminal block for permanent installation
- **Capacity**: 5A maximum (60W total system power)

### **Power Rails**
- **12V Rail**: Amplifier, LED strip (WS2815)
- **5V Rail**: Arduino, sensors, LED strip (WS2812B)
- **3.3V Rail**: Future expansion (onboard regulator)

### **Power Budget**
| Component | Voltage | Current | Power |
|-----------|---------|---------|-------|
| Arduino Uno | 5V | 50mA | 0.25W |
| HC-SR04 Sensors (2x) | 5V | 30mA | 0.15W |
| WWZMDiB XH-M543 | 12V | 2A | 24W |
| WS2812B LEDs (50x) | 5V | 1A | 5W |
| Status LEDs | 5V | 20mA | 0.1W |
| **Total** | - | **3.1A** | **29.5W** |

---

## 🔍 **Testing and Validation**

### **Pre-Assembly Tests**
1. **Continuity testing** - Verify all connections with multimeter
2. **Power rail testing** - Check voltage levels under load
3. **Signal integrity** - Verify digital signals with oscilloscope
4. **Connector testing** - Ensure proper mating and retention

### **Post-Assembly Tests**
1. **Power-on test** - Check status LEDs and power consumption
2. **Sensor functionality** - Verify ultrasonic readings
3. **Audio output** - Test amplifier with known signal
4. **LED control** - Verify FastLED library operation
5. **Waterproof testing** - Submersion test (if applicable)

---

## 📋 **Bill of Materials (BOM)**

### **Shield PCB Components**
| Component | Quantity | Part Number | Description |
|-----------|----------|-------------|-------------|
| PCB | 1 | Custom | 68.6mm x 53.4mm, 2-layer |
| Arduino Headers | 4 | - | Stackable 0.1" pitch |
| JST-XH Connectors | 3 | B4B-XH-A, B3B-XH-A | 4-pin (2x), 3-pin (1x) |
| Barrel Jack | 1 | PJ-202A | 2.1mm center positive |
| Terminal Blocks | 2 | - | 2-position, 5mm pitch |
| 3.5mm Jack | 1 | SJ-3523-SMT | Stereo audio |
| Status LEDs | 2 | - | 3mm, red/green |
| Resistors | 2 | - | 330Ω, 1/4W |
| Perfboard | 1 | - | 20x15 holes, 0.1" spacing |

### **External Components**
| Component | Quantity | Part Number | Description |
|-----------|----------|-------------|-------------|
| WWZMDiB XH-M543 | 1 | XH-M543 | 2x120W amplifier module |
| Hammond Enclosure | 1 | 1590WYF | 92x92x42mm waterproof |
| JST-XH Cables | 3 | - | Pre-wired with connectors |
| Cable Glands | 4 | - | M12 waterproof |
| Standoffs | 4 | - | M3 x 10mm nylon |

### **Estimated Total Cost**
- **Shield PCB + Components**: $25-30
- **WWZMDiB XH-M543**: $10-15  
- **Hammond Enclosure**: $15-20
- **Cables and Hardware**: $10-15
- **Total Project Cost**: **$60-80**

---

## 🚀 **Advanced Features**

### **Future Expansion Options**
- **I2C header** for additional sensors (temperature, humidity)
- **SPI header** for SD card logging
- **UART header** for wireless communication modules
- **Analog input expansion** via multiplexer IC
- **PWM output expansion** for motor control

### **Professional Modifications**
- **Custom PCB fabrication** for production runs
- **SMD components** for reduced size and cost
- **Integrated voltage regulators** for multiple power rails
- **EMI filtering** for improved audio quality
- **Conformal coating** for harsh environment protection

---

## 📖 **Usage with Solar Shrine Code**

This shield is designed to work seamlessly with the existing Solar Shrine codebase:

### **Compatible Arduino Sketches**
- ✅ `01_MAIN_SYSTEM/solar_shrine_theremin/solar_shrine_theremin.ino`
- ✅ `02_lighting_systems/fastled_dual_mode/fastled_dual_mode.ino`
- ✅ `03_audio_systems/speaker_test/speaker_test.ino`

### **Pin Mapping Compatibility** (Arduino Mega 2560)
All pin assignments match the updated Solar Shrine hardware configuration:
- **Sensors**: Pins 5,6,10,11 (updated for Mega)
- **LED Strip**: Pin 3 (unchanged)  
- **Audio Output**: Pin 12 → 1K resistor → Amplifier right channel (updated for Mega)
- **Power and Ground**: Standard Arduino rails

### **No Code Changes Required**
The shield maintains full compatibility with existing sketches - simply upload and run!

---

**🎯 This custom shield design provides a professional, waterproof, and compact solution for your Solar Shrine installation while maintaining full compatibility with your existing codebase and hardware requirements.** 