"""tempo_plugin.py - Plugin for Tempo-BT custom mcumgr commands"""
from smpclient import SMPClient
from smp import message as smp_message
from typing import Dict, Any, Optional, Tuple
import logging

logger = logging.getLogger(__name__)

class TempoCommands:
    """Handler for Tempo-BT custom commands (Group 64)"""
    
    GROUP_ID = 64  # Matches MGMT_GROUP_ID_TEMPO in your code
    
    def __init__(self, client: SMPClient):
        self.client = client
    
    def session_list(self) -> Dict[str, Any]:
        """List all logging sessions"""
        logger.info("Requesting session list (Group 64, Command 0)")
        
        request = smp_message.ReadRequest(
            group_id=self.GROUP_ID,
            command_id=0  # TEMPO_MGMT_ID_SESSION_LIST
        )
        
        response = self.client.request(request)
        logger.debug(f"Session list response: {response}")
        return response
    
    def session_info(self, session_id: str = None) -> Dict[str, Any]:
        """Get info about a specific session (not implemented yet)"""
        logger.info(f"Requesting session info for: {session_id}")
        
        # Your C code shows this is TODO, so it might return an error
        request = smp_message.ReadRequest(
            group_id=self.GROUP_ID,
            command_id=1  # TEMPO_MGMT_ID_SESSION_INFO
        )
        
        response = self.client.request(request)
        logger.debug(f"Session info response: {response}")
        return response
    
    def storage_info(self) -> Dict[str, Any]:
        """Get storage statistics"""
        logger.info("Requesting storage info (Group 64, Command 2)")
        
        request = smp_message.ReadRequest(
            group_id=self.GROUP_ID,
            command_id=2  # TEMPO_MGMT_ID_STORAGE_INFO
        )
        
        response = self.client.request(request)
        logger.debug(f"Storage info response: {response}")
        return response
    
    def led_control(self, enable: bool, r: int = 0, g: int = 0, b: int = 0) -> Dict[str, Any]:
        """Control the RGB LED
        
        Args:
            enable: True to turn on override, False to return to app control
            r: Red component (0-255)
            g: Green component (0-255)
            b: Blue component (0-255)
        
        Returns:
            Dict with current LED state
        """
        logger.info(f"LED control: enable={enable}, RGB=({r},{g},{b})")
        
        # Validate color values
        if not all(0 <= c <= 255 for c in [r, g, b]):
            raise ValueError("RGB values must be 0-255")
        
        payload = {
            "enable": enable,
            "r": r,
            "g": g,
            "b": b
        }
        
        request = smp_message.WriteRequest(
            group_id=self.GROUP_ID,
            command_id=3,  # TEMPO_MGMT_ID_LED_CONTROL
            data=payload
        )
        
        response = self.client.request(request)
        logger.debug(f"LED control response: {response}")
        return response
    
    def led_on(self, r: int, g: int, b: int) -> Dict[str, Any]:
        """Turn on LED with specified color"""
        return self.led_control(True, r, g, b)
    
    def led_off(self) -> Dict[str, Any]:
        """Turn off LED override (return to app control)"""
        return self.led_control(False)
    
    def led_preset(self, color: str) -> Dict[str, Any]:
        """Set LED to a preset color"""
        presets = {
            "red": (255, 0, 0),
            "green": (0, 255, 0),
            "blue": (0, 0, 255),
            "yellow": (255, 255, 0),
            "cyan": (0, 255, 255),
            "magenta": (255, 0, 255),
            "white": (255, 255, 255),
            "orange": (255, 128, 0),
        }
        
        if color.lower() not in presets:
            raise ValueError(f"Unknown color preset: {color}. Available: {', '.join(presets.keys())}")
        
        r, g, b = presets[color.lower()]
        return self.led_on(r, g, b)

# Required by smpmgr plugin system
def plugin_init(client: SMPClient) -> Dict[str, Any]:
    """Initialize plugin and return command handlers"""
    handler = TempoCommands(client)
    
    return {
        'tempo': {
            'session_list': handler.session_list,
            'session_info': handler.session_info,
            'storage_info': handler.storage_info,
            'led_control': handler.led_control,
            'led_on': handler.led_on,
            'led_off': handler.led_off,
            'led_preset': handler.led_preset,
        }
    }

# Helper function for CLI usage
def parse_rgb(rgb_string: str) -> Tuple[int, int, int]:
    """Parse RGB string like '255,0,0' or '#FF0000'"""
    if ',' in rgb_string:
        parts = rgb_string.split(',')
        if len(parts) != 3:
            raise ValueError("RGB format should be 'R,G,B'")
        return tuple(int(p.strip()) for p in parts)
    elif rgb_string.startswith('#'):
        hex_color = rgb_string.lstrip('#')
        if len(hex_color) != 6:
            raise ValueError("Hex color should be #RRGGBB")
        return tuple(int(hex_color[i:i+2], 16) for i in (0, 2, 4))
    else:
        raise ValueError("Unknown RGB format")