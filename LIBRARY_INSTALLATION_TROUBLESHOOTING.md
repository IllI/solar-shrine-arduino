# Library Installation Troubleshooting Guide

## Issue: NewTone Library Not Found During Compilation

### Problem Description
When compiling `solar_shrine_modular.ino`, you may encounter this error:
```
/path/to/robots.cpp:3:10: fatal error: NewTone.h: No such file or directory
 #include "NewTone.h"
          ^~~~~~~~~~~
compilation terminated.
exit status 1
Compilation error: NewTone.h: No such file or directory
```

### Root Causes
1. **Incorrect include syntax**: Using quotes `"NewTone.h"` instead of angle brackets `<NewTone.h>`
2. **Wrong library location**: Libraries in project folder instead of Arduino IDE's global libraries folder
3. **Arduino IDE not recognizing local libraries**: IDE expects libraries in specific system locations

## Solution Steps

### Step 1: Fix Include Statements in Code Files

#### Fix robots.cpp
**File**: `arduino/01_MAIN_SYSTEM/solar_shrine_modular/robots.cpp`

**Change line 3 from:**
```cpp
#include "NewTone.h"
```
**To:**
```cpp
#include <NewTone.h>
```

#### Fix theremin.cpp  
**File**: `arduino/01_MAIN_SYSTEM/solar_shrine_modular/theremin.cpp`

**Change line 2 from:**
```cpp
#include "NewTone.h"
```
**To:**
```cpp
#include <NewTone.h>
```

### Step 2: Move NewTone Library to Correct Location

#### For macOS Systems:

1. **Open Terminal**

2. **Navigate to your project directory:**
   ```bash
   cd /path/to/solar-shrine-arduino
   ```

3. **Copy NewTone library to Arduino's global libraries folder:**
   ```bash
   cp -r arduino/libraries/NewTone ~/Documents/Arduino/libraries/
   ```

4. **Verify the copy was successful:**
   ```bash
   ls -la ~/Documents/Arduino/libraries/NewTone
   ```
   
   You should see:
   ```
   total 24
   drwxr-xr-x@ 5 user  staff   160 [date] .
   drwxr-xr-x@ 8 user  staff   256 [date] ..
   -rw-r--r--@ 1 user  staff   443 [date] keywords.txt
   -rw-r--r--@ 1 user  staff  2583 [date] NewTone.cpp
   -rw-r--r--@ 1 user  staff  1939 [date] NewTone.h
   ```

### Step 3: Restart Arduino IDE

1. **Completely quit Arduino IDE** (Cmd+Q on Mac)
2. **Reopen Arduino IDE**
3. **Open your sketch** (`solar_shrine_modular.ino`)
4. **Try compiling** (Cmd+R or click verify button)

## Alternative Method: Install via Library Manager

If the above steps don't work, try installing NewTone through Arduino IDE:

1. **Open Arduino IDE**
2. **Go to Tools > Manage Libraries**
3. **Search for "NewTone"**
4. **Install the official NewTone library**

## Verification Checklist

Before compiling, verify:

- [ ] `robots.cpp` line 3 shows: `#include <NewTone.h>`
- [ ] `theremin.cpp` line 2 shows: `#include <NewTone.h>`
- [ ] NewTone library exists in: `~/Documents/Arduino/libraries/NewTone/`
- [ ] Arduino IDE has been restarted
- [ ] No build cache issues (delete build folders if needed)

## Quick Setup Script for New Mac Mini

Save this as `fix_newtone.sh` and run it:

```bash
#!/bin/bash

# Navigate to project directory (adjust path as needed)
cd /Users/antwane/Desktop/solar-shrine-arduino

echo "Fixing NewTone include statements..."

# Fix robots.cpp
sed -i '' 's/#include "NewTone.h"/#include <NewTone.h>/g' arduino/01_MAIN_SYSTEM/solar_shrine_modular/robots.cpp

# Fix theremin.cpp
sed -i '' 's/#include "NewTone.h"/#include <NewTone.h>/g' arduino/01_MAIN_SYSTEM/solar_shrine_modular/theremin.cpp

echo "Copying NewTone library to Arduino libraries folder..."

# Create Arduino libraries directory if it doesn't exist
mkdir -p ~/Documents/Arduino/libraries

# Copy NewTone library
cp -r arduino/libraries/NewTone ~/Documents/Arduino/libraries/

echo "Setup complete! Please restart Arduino IDE and try compiling."
echo "Verify the library was copied:"
ls -la ~/Documents/Arduino/libraries/NewTone
```

To use the script:
```bash
chmod +x fix_newtone.sh
./fix_newtone.sh
```

## Additional Libraries Setup

If you encounter similar issues with other libraries, ensure these are also in the Arduino libraries folder:

- **FastLED**: `~/Documents/Arduino/libraries/FastLED/`
- **ArduinoJson**: `~/Documents/Arduino/libraries/ArduinoJson/`
- **NewPing**: `~/Documents/Arduino/libraries/NewPing/`
- **Mozzi**: `~/Documents/Arduino/libraries/Mozzi/`
- **FixMath**: `~/Documents/Arduino/libraries/FixMath/`

Copy any missing libraries:
```bash
cp -r arduino/libraries/* ~/Documents/Arduino/libraries/
```

## Troubleshooting Tips

### If compilation still fails:

1. **Clear Arduino IDE cache:**
   - Quit Arduino IDE
   - Delete any `build` folders in your sketch directory
   - Restart Arduino IDE

2. **Check library structure:**
   ```bash
   ls ~/Documents/Arduino/libraries/NewTone/
   ```
   Should contain: `NewTone.h`, `NewTone.cpp`, `keywords.txt`

3. **Verify Arduino IDE preferences:**
   - Go to Arduino IDE > Preferences
   - Check "Sketchbook location" - libraries should be in `[location]/libraries/`

4. **Check for conflicting libraries:**
   - Ensure no duplicate NewTone libraries in different locations
   - Remove any NewTone libraries from project-local `libraries` folder after copying to global location

### Error Messages and Solutions:

| Error | Solution |
|-------|----------|
| `NewTone.h: No such file or directory` | Follow Steps 1-3 above |
| `multiple definition of...` | Remove duplicate libraries |
| `undefined reference to...` | Restart Arduino IDE, check library installation |

## System Requirements

- **Arduino IDE**: Version 1.8.0 or higher
- **macOS**: This guide is specific to macOS systems
- **Project**: Solar Shrine Arduino system

## Notes

- This fix needs to be applied once per Mac mini setup
- The script can be run multiple times safely
- Always restart Arduino IDE after library changes
- Keep a backup of your original files before making changes

---

**Last Updated**: August 2025
**Tested On**: macOS with Arduino IDE 1.8.x and 2.x 