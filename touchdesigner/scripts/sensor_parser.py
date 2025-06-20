# MODERN TOUCHDESIGNER JSON INTEGRATION - MULTIPLE APPROACHES
# Based on: https://interactiveimmersive.io/blog/python/getting-started-with-tdjson-in-touchdesigner/

# ==============================================================================
# APPROACH 1: JSON DAT (RECOMMENDED - NO COOK LOOPS)
# ==============================================================================
# 
# Setup: Serial DAT → JSON DAT → Effects
# 
# 1. Create JSON DAT, connect Serial DAT to it
# 2. JSON DAT settings:
#    - Filter: $ (gets all data)
#    - Output: Expression
# 3. Use expressions in your effect parameters:
#    - Left distance: op('json1').result['left']
#    - Right distance: op('json1').result['right'] 
#    - Hands detected: op('json1').result['hands_detected']
#
# Example effect parameter expressions:
# - Intensity: max(0, min(1, op('json1').result['left'] / 20.0))
# - Active: op('json1').result['hands_detected']
# - Color: op('json1').result['right'] / 20.0

# ==============================================================================
# APPROACH 2: TDJSON METHOD (ADVANCED)
# ==============================================================================

def onReceive(dat, rowIndex, message, byteData):
    """
    Advanced TDJSON approach using TouchDesigner's TDJSON module
    """
    # Skip if no data
    if dat.numRows == 0:
        return
    
    # Get the latest JSON data
    serial_data = dat[dat.numRows-1, 0].val.strip()
    if not serial_data:
        return
    
    # Parse and update using TDJSON
    update_effects_with_tdjson(serial_data)

def update_effects_with_tdjson(json_string):
    """
    Use TouchDesigner's TDJSON module to handle JSON data
    Based on Interactive & Immersive TDJSON guide
    """
    import json
    
    try:
        # Access TouchDesigner's TDJSON module
        TDJ = op.TDModules.mod.TDJSON
        
        # Parse the JSON string
        sensor_data = json.loads(json_string)
        
        # Update custom parameters on a settings component
        # This approach avoids cook loops by using custom parameters
        settings_comp = op('SETTINGS')  # Create a Base COMP called 'SETTINGS'
        
        if settings_comp:
            # Update custom parameters (create these in your SETTINGS component)
            if hasattr(settings_comp.par, 'Leftdistance'):
                settings_comp.par.Leftdistance = sensor_data.get('left', 0)
            if hasattr(settings_comp.par, 'Rightdistance'):
                settings_comp.par.Rightdistance = sensor_data.get('right', 0)
            if hasattr(settings_comp.par, 'Handsdetected'):
                settings_comp.par.Handsdetected = sensor_data.get('hands_detected', False)
        
        # Debug output
        print(f"TDJSON Updated: Left={sensor_data.get('left', 0)}, Right={sensor_data.get('right', 0)}, Hands={sensor_data.get('hands_detected', False)}")
        
    except Exception as e:
        print(f"TDJSON Error: {e}")

def create_preset_from_settings():
    """
    Create a preset using TDJSON (bonus feature)
    Based on Interactive & Immersive TDJSON tutorial
    """
    import json
    
    try:
        # Access TDJSON module
        TDJ = op.TDModules.mod.TDJSON
        
        # Convert settings component to JSON
        settings_comp = op('SETTINGS')
        if settings_comp:
            # Convert all custom parameters to JSON (including values)
            json_obj = TDJ.opToJSONOp(settings_comp, extraAttrs=['val'])
            
            # Save to a text DAT for storage
            preset_dat = op('current_preset')
            if preset_dat:
                preset_dat.text = json.dumps(json_obj)
                print("Preset saved to 'current_preset' DAT")
        
    except Exception as e:
        print(f"Preset creation error: {e}")

def load_preset_to_settings():
    """
    Load a preset using TDJSON
    """
    try:
        # Access TDJSON module
        TDJ = op.TDModules.mod.TDJSON
        
        # Load JSON from DAT
        preset_dat = op('current_preset')
        settings_comp = op('SETTINGS')
        
        if preset_dat and settings_comp:
            # Convert DAT back to JSON object
            json_obj = TDJ.datToJSON(preset_dat)
            
            # Apply parameters from JSON to settings component
            TDJ.addParametersFromJSONOp(settings_comp, json_obj)
            print("Preset loaded from 'current_preset' DAT")
            
    except Exception as e:
        print(f"Preset loading error: {e}")

# ==============================================================================
# SETUP INSTRUCTIONS
# ==============================================================================
# 
# OPTION 1 - JSON DAT (Easiest, Recommended):
# 1. Create JSON DAT, connect Serial DAT to it
# 2. Use expressions like op('json1').result['left'] in your effects
# 
# OPTION 2 - TDJSON Method:
# 1. Create a Base COMP called 'SETTINGS'
# 2. Add custom parameters to SETTINGS:
#    - Leftdistance (Float)
#    - Rightdistance (Float) 
#    - Handsdetected (Toggle)
# 3. Reference these in your effects: op('SETTINGS').par.Leftdistance.eval()
# 4. Optional: Create Text DATs called 'current_preset' for preset system
#
# EFFECT PARAMETER EXAMPLES:
# - Intensity: op('SETTINGS').par.Leftdistance.eval() / 20.0
# - Active: op('SETTINGS').par.Handsdetected.eval()
# - Color Mix: op('SETTINGS').par.Rightdistance.eval() / 20.0

# If you still want to use Python (not recommended), use this minimal approach:
def onReceive(dat, rowIndex, message, byteData):
    """
    Minimal callback - just store data, don't set parameters
    """
    pass  # Do nothing - let JSON DAT handle parsing

# me - this DAT
#
# dat - the DAT that received the data
# rowIndex - the row number the data was placed into
# message - an ascii representation of the data
#           Unprintable characters and unicode characters will
#           not be preserved. Use the 'byteData' parameter to get
#           the raw bytes that were sent.
# byteData - byte array of the data received
import json

def onReceive(dat, rowIndex, message, byteData):
    """
    Callback function for Serial DAT
    Fixed to avoid cook loop dependency errors
    """
    # Get the most recent line of serial data
    if dat.numRows > 0:
        serial_data = dat[dat.numRows-1, 0].val.strip()
    else:
        return
    
    # Skip empty lines
    if not serial_data:
        return
        
    # Parse the data
    sensor_data = parse_sensor_data(serial_data)
    
    if sensor_data:
        # Store data in a table DAT instead of setting parameters directly
        # This avoids cook loop issues
        store_sensor_data(sensor_data)
        
        # Use run() to defer parameter updates and avoid cook loops
        run("update_parameters()", delayFrames=1)
        
        # Debug output to textport
        print(f"Sensor Data: Left={sensor_data['left_distance']}, Right={sensor_data['right_distance']}, Hands={sensor_data['hands_detected']}")

def parse_sensor_data(serial_data):
    """
    Parse JSON data from Arduino serial input
    Arduino sends: {"left": 15, "right": 12, "hands_detected": true}
    Returns a dictionary with sensor values or None if parsing fails
    """
    try:
        # Parse the JSON string
        data = json.loads(serial_data)
        
        # Validate that required keys exist
        if 'left' not in data or 'right' not in data or 'hands_detected' not in data:
            print(f"Warning: Missing required keys in JSON: {serial_data}")
            return None
            
        return {
            'left_distance': float(data['left']),      # Arduino sends int, ensure float
            'right_distance': float(data['right']),    # Arduino sends int, ensure float  
            'hands_detected': bool(data['hands_detected'])  # Arduino sends bool
        }
    except json.JSONDecodeError as e:
        print(f"JSON Parse Error: {e} - Data: {serial_data}")
        return None
    except Exception as e:
        print(f"Unexpected error parsing sensor data: {e}")
        return None

def store_sensor_data(sensor_data):
    """
    Store sensor data in a Table DAT to avoid cook loop issues
    Create a table DAT called 'sensor_data' in the same component
    """
    try:
        # Try to find the sensor_data table DAT
        data_table = op('sensor_data')
        if not data_table:
            print("Warning: 'sensor_data' Table DAT not found. Create one to store sensor values.")
            return
            
        # Clear and set up the table
        data_table.clear()
        data_table.appendRow(['Parameter', 'Value'])
        data_table.appendRow(['left_distance', sensor_data['left_distance']])
        data_table.appendRow(['right_distance', sensor_data['right_distance']])
        data_table.appendRow(['hands_detected', sensor_data['hands_detected']])
        
    except Exception as e:
        print(f"Error storing sensor data: {e}")

def update_parameters():
    """
    Deferred parameter update function - called via run()
    This avoids cook loop dependency issues
    """
    try:
        # Get the sensor data from the table
        data_table = op('sensor_data')
        if not data_table or data_table.numRows < 4:
            return
            
        # Extract values from table
        left_distance = float(data_table[1, 1].val)
        right_distance = float(data_table[2, 1].val)
        hands_detected = data_table[3, 1].val == 'True'
        
        # Update parent component parameters safely
        parent = op('..')
        if hasattr(parent.par, 'left_distance'):
            parent.par.left_distance = left_distance
        if hasattr(parent.par, 'right_distance'):
            parent.par.right_distance = right_distance
        if hasattr(parent.par, 'hands_detected'):
            parent.par.hands_detected = hands_detected
            
        # Map to effects
        map_sensor_to_effects({
            'left_distance': left_distance,
            'right_distance': right_distance,
            'hands_detected': hands_detected
        })
        
    except Exception as e:
        print(f"Error updating parameters: {e}")

def map_sensor_to_effects(sensor_data):
    """
    Map sensor data to visual effects
    Only update effects if they exist to prevent errors
    """
    try:
        # Get references to the effects - use safe operator lookup
        left_effect = op('effects/left_effect')
        right_effect = op('effects/right_effect') 
        hands_effect = op('effects/hands_effect')
        
        # Map left sensor to effect (normalize from 0-20cm to 0-1)
        if left_effect and hasattr(left_effect.par, 'intensity'):
            normalized_left = max(0, min(1, sensor_data['left_distance'] / 20.0))
            left_effect.par.intensity = normalized_left
        
        # Map right sensor to effect (normalize from 0-20cm to 0-1)
        if right_effect and hasattr(right_effect.par, 'intensity'):
            normalized_right = max(0, min(1, sensor_data['right_distance'] / 20.0))
            right_effect.par.intensity = normalized_right
        
        # Map hands detected to effect
        if hands_effect and hasattr(hands_effect.par, 'active'):
            hands_effect.par.active = sensor_data['hands_detected']
            
    except Exception as e:
        print(f"Error mapping sensor data to effects: {e}")

# SETUP INSTRUCTIONS TO AVOID COOK LOOPS:
# 
# 1. Create a Table DAT called 'sensor_data' in the same component as this Serial DAT
# 2. The script will store parsed data in this table
# 3. Use expressions like: op('sensor_data')[1,1] to access left_distance
# 4. This approach avoids cook loop dependencies
#
# BETTER APPROACH: Use JSON DAT instead of this script:
# Serial DAT -> JSON DAT -> Reference values with op('json1').result['left']
# This eliminates cook loops entirely and follows modern TouchDesigner practices.

# Modern TouchDesigner JSON Handling Approach:
# 
# Instead of parsing JSON in Python, the current best practice is to use
# the JSON DAT operator which provides procedural data flow:
#
# 1. Serial DAT -> JSON DAT -> Your effects/parameters
# 2. In JSON DAT, set Filter parameter to extract specific values:
#    - For left distance: $.left
#    - For right distance: $.right  
#    - For hands detected: $.hands_detected
# 3. Use expressions like: op('json1').result to access parsed values
# 4. Set JSON DAT Output Format to "Table" for tabular data output
#
# This approach eliminates scripting and keeps data flow visual and procedural.
# See: https://docs.derivative.ca/JSON_DAT

# COOK-LOOP-FREE JSON PARSING - Based on TouchDesigner Forum Best Practices
# Pattern from: https://forum.derivative.ca/t/render-pick-chop-cook-dependency-loop/139180
# Expert: Matthew Ragan's approach using Table CHOPs

import json

def onReceive(dat, rowIndex, message, byteData):
    """
    Minimal callback that writes to Table CHOP - NO COOK LOOPS
    Based on Matthew Ragan's forum solution
    """
    # Get the latest JSON line
    if dat.numRows > 0:
        json_string = dat[dat.numRows-1, 0].val.strip()
        
        # Skip empty lines
        if not json_string:
            return
            
        # Parse and write to Table CHOP (cook-loop-free)
        parse_to_table_chop(json_string)

def parse_to_table_chop(json_string):
    """
    Parse JSON and write to Table CHOP - avoids all cook loops
    """
    try:
        # Parse JSON
        data = json.loads(json_string)
        
        # Write to Table CHOP (create a Table CHOP called 'sensor_table')
        table_chop = op('sensor_table')
        if table_chop:
            # Set values directly in Table CHOP channels
            table_chop[0, 'left'] = data.get('left', 0)
            table_chop[0, 'right'] = data.get('right', 0)
            table_chop[0, 'hands'] = 1 if data.get('hands_detected', False) else 0
            
            # Debug
            print(f"Updated Table CHOP: Left={data.get('left', 0)}, Right={data.get('right', 0)}, Hands={data.get('hands_detected', False)}")
    
    except json.JSONDecodeError as e:
        print(f"JSON Error: {e}")
    except Exception as e:
        print(f"Table CHOP Error: {e}")

# ALTERNATIVE: Pure JSON DAT approach (if Table CHOP doesn't work)
def onReceive_minimal(dat, rowIndex, message, byteData):
    """
    Absolute minimal approach - just pass data through
    """
    pass  # Do nothing - let JSON DAT handle everything

# ==============================================================================
# SETUP INSTRUCTIONS - PROVEN WORKING CONFIGURATION
# ==============================================================================
# 
# 1. CLEAR ALL CALLBACKS from your Serial DAT (leave callbacks empty)
# 
# 2. CREATE SELECT DAT called 'select1':
#    Connect: Serial DAT → Select DAT
#    Configure Select DAT with these EXACT settings:
#    
#    ROWS SECTION:
#    - Include First Row: OFF
#    - Select Rows: by Index
#    - Start Row Index: 10 (or desired row number)
#    - End Row Index: 10 (same as start for single row)
#    - Row Select Values: + (enabled)
#    - Row Select Condition: 10
#    - From Column: 0
#    
#    COLUMNS SECTION:
#    - Include First Col: OFF
#    - Select Cols: by Index  
#    - Start Col Index: 0
#    - End Col Index: 0
#    - Col Select Values: + (enabled)
#    - Col Select Condition: (leave empty)
#    - From Row: 0
#    
#    OUTPUT:
#    - Output: Input Data
#
# 3. CREATE JSON DAT called 'json1':
#    Connect: Select DAT → JSON DAT
#    Configure JSON DAT:
#    - Filter: $ (gets all data)
#    - Output: Expression
#
# 4. FINAL DATA CHAIN:
#    Serial DAT → Select DAT → JSON DAT → Your Effects
#
# 5. ACCESS DATA IN EFFECTS:
#    - Left distance: op('json1').result['left']
#    - Right distance: op('json1').result['right']  
#    - Hands detected: op('json1').result['hands_detected']
#
# ==============================================================================
# DYNAMIC ROW SELECTION (Advanced)
# ==============================================================================
# 
# To always get the LATEST row instead of row 10:
# In Select DAT Row Select Condition, use:
# me.inputs[0].numRows - 1
# 
# This will dynamically select the last row as new data comes in.
# 
# ==============================================================================
# EFFECT EXAMPLES WITH HANDS DETECTION
# ==============================================================================
# 
# Conditional effects (only active when hands detected):
# 
# Intensity Parameter:
# max(0, min(1, op('json1').result['left'] / 200.0)) if op('json1').result['hands_detected'] else 0
# 
# Color Mix Parameter:
# op('json1').result['right'] / 200.0 if op('json1').result['hands_detected'] else 0.5
# 
# Boolean Toggle:
# op('json1').result['hands_detected']
# 
# Scaled Value with Gating:
# (op('json1').result['left'] / 200.0) * op('json1').result['hands_detected']

# ==============================================================================
# WHY THIS SETUP WORKS:
# ==============================================================================
# - No Python callbacks = No cook dependency loops
# - Select DAT extracts single row with clean JSON
# - JSON DAT parses the single JSON object correctly  
# - Pure procedural data flow that TouchDesigner handles efficiently
# - Real-time updates as Arduino sends new sensor data

# If you absolutely need Python, use this minimal Execute DAT approach:
def onTableChange(dat):
    """
    Use this in an Execute DAT (not Serial DAT callbacks)
    Connect Execute DAT input to Serial DAT
    """
    if dat.numRows > 0:
        # Get latest JSON line
        latest = dat[dat.numRows-1, 0].val.strip()
        
        # Write to Text DAT
        text_dat = op('clean_json')
        if text_dat and latest:
            text_dat.text = latest

	