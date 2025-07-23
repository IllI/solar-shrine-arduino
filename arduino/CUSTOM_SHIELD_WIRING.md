# Solar Shrine - Custom Shield Wiring Guide

This document details the wiring of the custom protoboard shield used with the Arduino Mega 2560 for the Solar Shrine project.

## Image Gallery

The following images show the project setup and the specific wiring of the custom shield.

### Figure 1: Project Overview
*This image shows the complete test bench, including the Arduino Mega, the custom shield, ultrasonic sensors, audio exciters, and the hand-shaped LED strip.*
![Project Overview](shield_wiring_2.jpeg)

### Figure 2: Shield Top View
*This is a close-up of the shield mounted on the Arduino Mega, showing the primary connectors for the sensors and the LED strip.*
![Shield Top View](shield_wiring.jpeg)

### Figure 3: Shield Bottom View (Wiring Details)
*This image shows the underside of the protoshield, detailing the point-to-point solder connections for all components.*
![Shield Bottom View](shield_wiring_3.jpeg)

## Shield Wiring Analysis

Based on the images, the custom protoshield provides dedicated connections for the ultrasonic sensors, the LED strip, and the audio output. The wiring has been implemented to match the Solar Shrine Modular System's pinout on the Arduino Mega.

### Pin Connections and Hardware

*   **Left Ultrasonic Sensor (4-pin JST-style connector):**
    *   **VCC:** Connects to the 5V rail.
    *   **GND:** Connects to the GND rail.
    *   **Trig:** Wired to **Pin 10**.
    *   **Echo:** Wired to **Pin 11**.

*   **Right Ultrasonic Sensor (4-pin JST-style connector):**
    *   **VCC:** Connects to the 5V rail.
    *   **GND:** Connects to the GND rail.
    *   **Trig:** Wired to **Pin 5**.
    *   **Echo:** Wired to **Pin 6**.

*   **LED Strip (3-pin JST-style connector):**
    *   **VCC:** Connects to the 5V rail (for WS2812B LEDs).
    *   **GND:** Connects to the GND rail.
    *   **Data:** Wired to **Pin 3**.

*   **Audio Output:**
    *   A **1K Ohm resistor** is soldered between **Pin 12** and the audio output line. This is the current-limiting resistor for the amplifier.
    *   The connection appears to be a 2-pin header for the audio output and ground.
        *   **Signal:** From the 1K resistor connected to **Pin 12**.
        *   **Ground:** Connects to the GND rail.

### Power Distribution

*   The shield uses the Arduino's onboard `5V` and `GND` pins to create power and ground rails.
*   These rails are then used to supply power to both ultrasonic sensors and the LED strip, which is a clean and effective way to manage power for these components.

### Shield Construction Notes

*   The shield is built on a standard "Uno size" protoboard, making it compatible with both Uno and Mega form factors.
*   The use of JST-style connectors provides a secure and reliable way to connect and disconnect the external components.
*   The point-to-point soldering is done with enameled copper wire, which is a common technique for creating permanent, custom circuits.

This custom shield is a robust and well-designed piece of hardware that perfectly complements the modular software system. 