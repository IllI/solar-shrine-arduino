"""
Sensor Parser for Solar Shrine
This script parses JSON data from Arduino sensors and makes it available to TouchDesigner
"""

import json

def parse_sensor_data(serial_data):
    """
    Parse JSON data from Arduino serial input
    Returns a dictionary with sensor values or None if parsing fails
    """
    try:
        data = json.loads(serial_data)
        return {
            'left_distance': data['left'],
            'right_distance': data['right'],
            'hands_detected': data['hands_detected']
        }
    except:
        return None

def onSerialData(dat):
    """
    Callback function for Serial DAT
    """
    # Get the serial data
    serial_data = dat.text
    
    # Parse the data
    sensor_data = parse_sensor_data(serial_data)
    
    if sensor_data:
        # Update parameters in the parent component
        parent = op('..')
        parent.par.left_distance = sensor_data['left_distance']
        parent.par.right_distance = sensor_data['right_distance']
        parent.par.hands_detected = sensor_data['hands_detected']
        
        # You can add more processing here
        # For example, mapping the values to visual effects
        map_sensor_to_effects(sensor_data)

def map_sensor_to_effects(sensor_data):
    """
    Map sensor data to visual effects
    """
    # Get references to the effects
    left_effect = op('effects/left_effect')
    right_effect = op('effects/right_effect')
    hands_effect = op('effects/hands_effect')
    
    # Map left sensor to effect
    left_effect.par.intensity = sensor_data['left_distance'] / 20.0  # Normalize to 0-1
    
    # Map right sensor to effect
    right_effect.par.intensity = sensor_data['right_distance'] / 20.0  # Normalize to 0-1
    
    # Map hands detected to effect
    hands_effect.par.active = sensor_data['hands_detected'] 