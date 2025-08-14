#!/bin/bash

# Solar Shrine - NewTone Library Fix Script
# This script fixes the NewTone library compilation issues
# Run this on each Mac mini that needs the Solar Shrine setup

echo "🔧 Solar Shrine - NewTone Library Fix Script"
echo "=============================================="

# Get the current directory
CURRENT_DIR=$(pwd)
echo "📁 Current directory: $CURRENT_DIR"

# Check if we're in the right directory
if [[ ! -d "arduino/01_MAIN_SYSTEM/solar_shrine_modular" ]]; then
    echo "❌ Error: Not in the solar-shrine-arduino project directory"
    echo "Please navigate to the solar-shrine-arduino directory first:"
    echo "cd /path/to/solar-shrine-arduino"
    exit 1
fi

echo "✅ Found Solar Shrine project structure"

echo ""
echo "🔄 Step 1: Fixing NewTone include statements..."

# Fix robots.cpp
ROBOTS_FILE="arduino/01_MAIN_SYSTEM/solar_shrine_modular/robots.cpp"
if [[ -f "$ROBOTS_FILE" ]]; then
    echo "  📝 Fixing $ROBOTS_FILE"
    sed -i '' 's/#include "NewTone\.h"/#include <NewTone.h>/g' "$ROBOTS_FILE"
    echo "  ✅ Fixed robots.cpp"
else
    echo "  ⚠️  Warning: $ROBOTS_FILE not found"
fi

# Fix theremin.cpp
THEREMIN_FILE="arduino/01_MAIN_SYSTEM/solar_shrine_modular/theremin.cpp"
if [[ -f "$THEREMIN_FILE" ]]; then
    echo "  📝 Fixing $THEREMIN_FILE"
    sed -i '' 's/#include "NewTone\.h"/#include <NewTone.h>/g' "$THEREMIN_FILE"
    echo "  ✅ Fixed theremin.cpp"
else
    echo "  ⚠️  Warning: $THEREMIN_FILE not found"
fi

echo ""
echo "📚 Step 2: Setting up Arduino libraries..."

# Create Arduino libraries directory if it doesn't exist
ARDUINO_LIBS_DIR="$HOME/Documents/Arduino/libraries"
echo "  📁 Arduino libraries directory: $ARDUINO_LIBS_DIR"

if [[ ! -d "$ARDUINO_LIBS_DIR" ]]; then
    echo "  📁 Creating Arduino libraries directory..."
    mkdir -p "$ARDUINO_LIBS_DIR"
fi

# Check if NewTone library exists in project
PROJECT_NEWTONE="arduino/libraries/NewTone"
if [[ -d "$PROJECT_NEWTONE" ]]; then
    echo "  📦 Found NewTone library in project"
    echo "  📋 Copying NewTone library to Arduino libraries folder..."
    cp -r "$PROJECT_NEWTONE" "$ARDUINO_LIBS_DIR/"
    echo "  ✅ NewTone library copied successfully"
else
    echo "  ❌ Error: NewTone library not found in arduino/libraries/NewTone"
    echo "  Please ensure the NewTone library is present in the project"
    exit 1
fi

# Copy all other libraries as well (in case they're needed)
echo ""
echo "📦 Step 3: Copying all project libraries to Arduino libraries folder..."

for lib_dir in arduino/libraries/*/; do
    if [[ -d "$lib_dir" ]]; then
        lib_name=$(basename "$lib_dir")
        echo "  📋 Copying $lib_name..."
        cp -r "$lib_dir" "$ARDUINO_LIBS_DIR/"
    fi
done

echo "  ✅ All libraries copied"

echo ""
echo "🔍 Step 4: Verification..."

# Verify NewTone library structure
NEWTONE_TARGET="$ARDUINO_LIBS_DIR/NewTone"
if [[ -f "$NEWTONE_TARGET/NewTone.h" && -f "$NEWTONE_TARGET/NewTone.cpp" ]]; then
    echo "  ✅ NewTone library structure verified"
    echo "  📁 Contents of $NEWTONE_TARGET:"
    ls -la "$NEWTONE_TARGET"
else
    echo "  ❌ Error: NewTone library not properly installed"
    exit 1
fi

# Check the fixed include statements
echo ""
echo "  🔍 Verifying include statement fixes..."

if grep -q '#include <NewTone.h>' "$ROBOTS_FILE" 2>/dev/null; then
    echo "  ✅ robots.cpp: Include statement fixed"
else
    echo "  ❌ robots.cpp: Include statement not fixed"
fi

if grep -q '#include <NewTone.h>' "$THEREMIN_FILE" 2>/dev/null; then
    echo "  ✅ theremin.cpp: Include statement fixed"
else
    echo "  ❌ theremin.cpp: Include statement not fixed"
fi

echo ""
echo "🎉 Setup Complete!"
echo "==================="
echo ""
echo "📋 Next Steps:"
echo "1. 🔄 Completely quit Arduino IDE (Cmd+Q)"
echo "2. 🚀 Reopen Arduino IDE"
echo "3. 📂 Open solar_shrine_modular.ino"
echo "4. ⚡ Try compiling (Cmd+R or click verify button)"
echo ""
echo "📍 Libraries installed in: $ARDUINO_LIBS_DIR"
echo ""
echo "🆘 If you still get compilation errors:"
echo "   • Make sure Arduino IDE is completely restarted"
echo "   • Delete any 'build' folders in the sketch directory"
echo "   • Check Arduino IDE > Preferences > Sketchbook location"
echo ""
echo "✨ The Solar Shrine should now compile successfully!" 