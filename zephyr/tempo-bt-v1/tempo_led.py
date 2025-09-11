#!/usr/bin/env python3
"""
tempo_led.py - Simple LED control for Tempo-BT

Usage:
    ./tempo_led.py red              # Turn on red
    ./tempo_led.py 255,128,0        # Orange using RGB values
    ./tempo_led.py "#FF8000"        # Orange using hex
    ./tempo_led.py off              # Turn off override
"""

import sys
import subprocess
import argparse

# Color presets
PRESETS = {
    "red": "255,0,0",
    "green": "0,255,0", 
    "blue": "0,0,255",
    "yellow": "255,255,0",
    "cyan": "0,255,255",
    "magenta": "255,0,255",
    "white": "255,255,255",
    "orange": "255,128,0",
    "off": None
}

def parse_color(color_str):
    """Parse color string and return R,G,B values"""
    # Check for presets
    if color_str.lower() in PRESETS:
        if color_str.lower() == "off":
            return None
        color_str = PRESETS[color_str.lower()]
    
    # Parse hex color
    if color_str.startswith('#'):
        hex_color = color_str.lstrip('#')
        if len(hex_color) != 6:
            raise ValueError("Hex color should be #RRGGBB")
        r = int(hex_color[0:2], 16)
        g = int(hex_color[2:4], 16)
        b = int(hex_color[4:6], 16)
        return r, g, b
    
    # Parse RGB values
    if ',' in color_str:
        parts = color_str.split(',')
        if len(parts) != 3:
            raise ValueError("RGB format should be R,G,B")
        r, g, b = [int(p.strip()) for p in parts]
        if not all(0 <= c <= 255 for c in [r, g, b]):
            raise ValueError("RGB values must be 0-255")
        return r, g, b
    
    raise ValueError(f"Unknown color format: {color_str}")

def main():
    parser = argparse.ArgumentParser(description="Control Tempo-BT RGB LED")
    parser.add_argument("color", help="Color (preset name, R,G,B, #RRGGBB, or 'off')")
    parser.add_argument("--device", default="Tempo-BT", help="Device name or MAC")
    parser.add_argument("--plugin", default="tempo_plugin.py", help="Plugin path")
    parser.add_argument("-v", "--verbose", action="store_true", help="Verbose output")
    
    args = parser.parse_args()
    
    try:
        color = parse_color(args.color)
    except ValueError as e:
        print(f"Error: {e}")
        print(f"Available presets: {', '.join(PRESETS.keys())}")
        return 1
    
    # Build smpmgr command
    cmd = ["smpmgr", "--conn", "ble", "--device-name", args.device, 
           "--plugin", args.plugin, "tempo"]
    
    if args.verbose:
        cmd.insert(1, "-vv")
    
    if color is None:
        # Turn off
        cmd.extend(["led_off"])
        print("Turning off LED override...")
    else:
        # Turn on with color
        r, g, b = color
        cmd.extend(["led_on", str(r), str(g), str(b)])
        print(f"Setting LED to RGB({r},{g},{b})...")
    
    # Execute command
    try:
        result = subprocess.run(cmd, capture_output=True, text=True)
        if result.returncode == 0:
            print("Success!")
            if args.verbose:
                print(result.stdout)
        else:
            print(f"Error: Command failed")
            print(result.stderr)
            return 1
    except Exception as e:
        print(f"Error running smpmgr: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    sys.exit(main())