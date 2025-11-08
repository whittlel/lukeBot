"""
Arduino Interface for USB Serial communication.
Handles high-level commands sent to Arduino Mega.
"""

import serial
import serial.tools.list_ports
import time
import threading
from typing import Optional


class ArduinoInterface:
    """Interface for communicating with Arduino Mega via USB Serial."""
    
    def __init__(self, port=None, baud_rate=115200, timeout=1.0):
        """
        Initialize Arduino interface.
        
        Args:
            port: Serial port (None for auto-detect)
            baud_rate: Serial baud rate (default: 115200)
            timeout: Serial timeout in seconds
        """
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.serial_connection = None
        self.connected = False
        self.lock = threading.Lock()
    
    def find_arduino_port(self):
        """Auto-detect Arduino port."""
        ports = serial.tools.list_ports.comports()
        for port in ports:
            # Common Arduino identifiers
            if any(keyword in port.description.lower() for keyword in 
                   ['arduino', 'mega', 'usb serial', 'ch340', 'cp210', 'ftdi']):
                return port.device
            
            # Also check by VID/PID (common Arduino IDs)
            if hasattr(port, 'vid') and hasattr(port, 'pid'):
                # Arduino Mega: 0x2341:0x0010 (Arduino LLC), 0x2341:0x0042, etc.
                if port.vid in [0x2341, 0x2A03]:  # Arduino LLC or Arduino.org
                    return port.device
        
        return None
    
    def connect(self):
        """Connect to Arduino."""
        if self.connected:
            return True
        
        try:
            # Auto-detect port if not specified
            if self.port is None:
                self.port = self.find_arduino_port()
                if self.port is None:
                    print("[ERROR] Could not find Arduino port")
                    return False
            
            # Open serial connection
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=self.timeout,
                write_timeout=self.timeout
            )
            
            # Wait for Arduino to initialize
            time.sleep(2.0)
            
            # Clear any pending data
            self.serial_connection.reset_input_buffer()
            self.serial_connection.reset_output_buffer()
            
            self.connected = True
            print(f"[INFO] Connected to Arduino on {self.port}")
            return True
            
        except serial.SerialException as e:
            print(f"[ERROR] Failed to connect to Arduino: {e}")
            self.connected = False
            return False
        except Exception as e:
            print(f"[ERROR] Unexpected error connecting to Arduino: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """Disconnect from Arduino."""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
        self.connected = False
        self.serial_connection = None
        print("[INFO] Disconnected from Arduino")
    
    def send_command(self, command: str) -> bool:
        """
        Send command to Arduino.
        
        Args:
            command: Command string (e.g., "MOVE_FORWARD,50")
        
        Returns:
            True if command sent successfully
        """
        if not self.connected or not self.serial_connection:
            print("[ERROR] Not connected to Arduino")
            return False
        
        try:
            with self.lock:
                # Send command with newline
                command_bytes = (command + '\n').encode('utf-8')
                self.serial_connection.write(command_bytes)
                self.serial_connection.flush()
                return True
        except serial.SerialException as e:
            print(f"[ERROR] Failed to send command: {e}")
            self.connected = False
            return False
        except Exception as e:
            print(f"[ERROR] Unexpected error sending command: {e}")
            return False
    
    def read_response(self, timeout=None) -> Optional[str]:
        """
        Read response from Arduino.
        
        Args:
            timeout: Timeout in seconds (None uses default)
        
        Returns:
            Response string or None if timeout
        """
        if not self.connected or not self.serial_connection:
            return None
        
        try:
            old_timeout = self.serial_connection.timeout
            if timeout is not None:
                self.serial_connection.timeout = timeout
            
            with self.lock:
                if self.serial_connection.in_waiting > 0:
                    response = self.serial_connection.readline().decode('utf-8').strip()
                    self.serial_connection.timeout = old_timeout
                    return response
                
                self.serial_connection.timeout = old_timeout
                return None
        except Exception as e:
            print(f"[ERROR] Error reading response: {e}")
            return None
    
    def move_forward(self, speed: int) -> bool:
        """Send move forward command."""
        if speed < 0 or speed > 100:
            print(f"[ERROR] Speed must be between 0 and 100, got {speed}")
            return False
        return self.send_command(f"MOVE_FORWARD,{speed}")
    
    def move_backward(self, speed: int) -> bool:
        """Send move backward command."""
        if speed < 0 or speed > 100:
            print(f"[ERROR] Speed must be between 0 and 100, got {speed}")
            return False
        return self.send_command(f"MOVE_BACKWARD,{speed}")
    
    def turn_left(self, angle: float) -> bool:
        """Send turn left command."""
        return self.send_command(f"TURN_LEFT,{angle}")
    
    def turn_right(self, angle: float) -> bool:
        """Send turn right command."""
        return self.send_command(f"TURN_RIGHT,{angle}")
    
    def stop(self) -> bool:
        """Send stop command."""
        return self.send_command("STOP")
    
    def is_connected(self) -> bool:
        """Check if connected to Arduino."""
        return self.connected and self.serial_connection is not None and self.serial_connection.is_open

