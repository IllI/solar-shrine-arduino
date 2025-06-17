"""
Movie Integration Parser for Po Tolo Project
Simplified Arduino sensor integration for movie-based TouchDesigner projects
"""

import json

def parse_arduino_data(serial_dat):
    """
    Simple function to parse Arduino JSON and return usable values
    Call this from a DAT Execute or Python CHOP
    """
    if serial_dat.numRows == 0:
        return None
        
    try:
        # Get the latest line of serial data
        latest_data = serial_dat[serial_dat.numRows-1, 0].val
        
        # Parse JSON
        sensor_data = json.loads(latest_data)
        
        return {
            'left': sensor_data.get('left', 0),
            'right': sensor_data.get('right', 0), 
            'hands_detected': sensor_data.get('hands_detected', False),
            'left_normalized': min(sensor_data.get('left', 0) / 20.0, 1.0),  # 0-1 range
            'right_normalized': min(sensor_data.get('right', 0) / 20.0, 1.0),  # 0-1 range
        }
    except:
        return None

# Example usage in DAT Execute callback:
def onTableChange(dat):
    """
    Put this function in your DAT Execute that monitors the Serial DAT
    """
    data = parse_arduino_data(dat)
    
    if data:
        # Update movie parameters based on sensor data
        movie_node = op('/project1/moviefilein1')  # Adjust path to your movie node
        
        # Example effects:
        
        # 1. Control playback speed with left sensor
        if movie_node:
            speed = 0.5 + (data['left_normalized'] * 1.5)  # Speed range: 0.5 to 2.0
            movie_node.par.speed = speed
        
        # 2. Control opacity with right sensor  
        # (You'll need to add this to a material or composite)
        
        # 3. Trigger different behavior when hands detected
        if data['hands_detected']:
            # Hands detected - special mode
            print("Hands detected! Triggering special effects...")
            # Add your special effects here
        else:
            # Normal mode
            pass
            
        # Store values in a global for other nodes to access
        parent().store('arduino_left', data['left'])
        parent().store('arduino_right', data['right'])
        parent().store('arduino_hands', data['hands_detected'])

# Alternative: Simple CHOP Script version
def onCook(scriptOp):
    """
    Use this in a Python CHOP if you prefer CHOP-based approach
    """
    # Get serial data
    serial_dat = op('../serial_arduino')
    data = parse_arduino_data(serial_dat)
    
    if data:
        # Create CHOP channels
        scriptOp.clear()
        scriptOp.appendChan('left_distance')
        scriptOp.appendChan('right_distance') 
        scriptOp.appendChan('hands_detected')
        scriptOp.appendChan('left_norm')
        scriptOp.appendChan('right_norm')
        
        # Set values
        scriptOp['left_distance'][0] = data['left']
        scriptOp['right_distance'][0] = data['right']
        scriptOp['hands_detected'][0] = 1 if data['hands_detected'] else 0
        scriptOp['left_norm'][0] = data['left_normalized']
        scriptOp['right_norm'][0] = data['right_normalized'] 