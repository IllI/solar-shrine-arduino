# Solar Shrine - Sensor Troubleshooting Guide

## 🔍 **Common Sensor Issues and Solutions**

This guide documents solutions to sensor problems encountered during development.

---

## ⚠️ **Issue #1: Sensor Reads 0 Consistently**

### **Symptoms:**
- Sensor works perfectly in standalone test scripts
- Same sensor returns 0 in main system
- Hardware and wiring are confirmed working
- Other sensors on different pins work fine

### **Root Cause:**
Missing `pinMode()` setup in the main system's `setup()` function.

### **Solution:**
Add proper pin configuration in `setup()`:

```cpp
void setup() {
  // ... other setup code ...
  
  // CRITICAL: Set up sensor pins
  pinMode(trigPin1, OUTPUT);  // Trigger pin MUST be OUTPUT
  pinMode(echoPin1, INPUT);   // Echo pin MUST be INPUT
  pinMode(trigPin2, OUTPUT);  // Second sensor trigger
  pinMode(echoPin2, INPUT);   // Second sensor echo
  
  // ... rest of setup ...
}
```

### **Why This Happens:**
- Arduino pins default to INPUT mode
- `digitalWrite()` on INPUT pins has no effect
- Sensor never receives trigger pulse
- Always returns timeout (0)

### **Prevention:**
Always verify pin setup when integrating sensors into complex systems.

---

## ⚠️ **Issue #2: HC-SR04 "Stuck Echo" Problem**

### **Symptoms:**
- Sensor works initially, then stops responding
- Echo pin remains HIGH permanently
- Requires power cycle to work again
- Affects only certain HC-SR04 variants

### **Root Cause:**
Some HC-SR04 sensors (Version B) don't auto-reset when no echo is detected.

### **Solution:**
Implement reset fix in sensor reading function:

```cpp
float readDistanceWithReset(int trigPin, int echoPin) {
  // Check if echo pin is stuck HIGH
  if (digitalRead(echoPin) == HIGH) {
    // Apply reset fix
    pinMode(echoPin, OUTPUT);
    digitalWrite(echoPin, LOW);
    delayMicroseconds(10);
    pinMode(echoPin, INPUT);
    delayMicroseconds(10);
  }
  
  // Normal sensor reading
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  unsigned long duration = pulseIn(echoPin, HIGH, 50000UL);
  
  if (duration == 0) {
    return 0; // No echo detected
  }
  
  return (duration * 0.0343) / 2.0;
}
```

### **Affected Hardware:**
- Cheap HC-SR04 variants from certain manufacturers
- Usually identifiable by extremely low cost (<$1)
- Same part number as working versions

### **Reference:**
Based on research from mc-computing.com HC-SR04 troubleshooting guide.

---

## ⚠️ **Issue #3: NewPing + Mozzi Library Conflicts**

### **Symptoms:**
- Sensors work with NewPing library alone
- Fail when Mozzi audio library is active
- Timer-related interference

### **Root Cause:**
- Mozzi uses Timer1 for audio at 16kHz
- NewPing library timing can be disrupted
- Particularly affects first sensor in some cases

### **Solution:**
Use direct `pulseIn()` method instead of NewPing for affected sensors:

```cpp
// Instead of: NewPing sonar1(trigPin1, echoPin1, 200);
// Use direct method with reset fix (see Issue #2)

float distance1 = readDistanceWithReset(trigPin1, echoPin1);
```

### **When to Apply:**
- Only when NewPing + Mozzi combination fails
- Can mix approaches: problematic sensor uses direct method, working sensors keep NewPing

---

## 🔧 **Diagnostic Steps**

### **Step 1: Verify Hardware**
1. Test sensor with standalone script (`05_basic_sensors/sensor_troubleshoot/`)
2. Confirm wiring with multimeter
3. Check power supply (5V, adequate current)

### **Step 2: Check Pin Setup**
1. Verify `pinMode()` calls in `setup()`
2. Confirm pin numbers match hardware
3. Test with simple digitalWrite/digitalRead

### **Step 3: Test Library Conflicts**
1. Temporarily disable Mozzi (`#define ENABLE_AUDIO false`)
2. Test with NewPing library
3. If works, apply direct method solution

### **Step 4: Apply Fixes**
1. Add missing pin setup
2. Implement reset fix for stuck sensors
3. Use direct method for library conflicts

---

## 📋 **Quick Reference**

### **Essential Pin Setup:**
```cpp
pinMode(trigPin, OUTPUT);  // Always required
pinMode(echoPin, INPUT);   // Always required
```

### **Reset Fix Pattern:**
```cpp
if (digitalRead(echoPin) == HIGH) {
  pinMode(echoPin, OUTPUT);
  digitalWrite(echoPin, LOW);
  delayMicroseconds(10);
  pinMode(echoPin, INPUT);
  delayMicroseconds(10);
}
```

### **Direct Reading Method:**
```cpp
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
unsigned long duration = pulseIn(echoPin, HIGH, 50000UL);
```

---

## 🎯 **Prevention Checklist**

- [ ] Pin setup in `setup()` function
- [ ] Test sensors individually before integration
- [ ] Document which sensors need reset fix
- [ ] Note any library conflicts
- [ ] Keep troubleshooting scripts for testing

---

**Created:** January 2025  
**Last Updated:** January 2025  
**Issue Resolution:** HC-SR04 sensor reading 0 in main system - missing pinMode() setup 