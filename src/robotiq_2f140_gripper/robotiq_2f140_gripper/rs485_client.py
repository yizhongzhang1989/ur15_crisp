#!/usr/bin/env python3
"""
RS485/Modbus communication client over TCP/IP socket.
"""

import socket
import time
import threading
from typing import List, Optional, Union


class RS485Client:
    """
    RS485/Modbus communication client over TCP/IP socket.
    Provides easy-to-use methods for sending commands and receiving responses.
    """
    
    def __init__(self, host: str, port: int, timeout: float = 1.0):
        """
        Initialize RS485 client.
        
        Args:
            host: IP address of the RS485 gateway
            port: Port number for RS485 communication
            timeout: Socket timeout in seconds (default: 1.0)
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.socket: Optional[socket.socket] = None
        self._connected = False
        self._lock = threading.Lock()  # Thread safety for concurrent access
    
    def connect(self, max_retries: int = 3, retry_delay: float = 0.5) -> bool:
        """
        Establish connection to RS485 gateway with retry logic.
        
        Args:
            max_retries: Maximum number of connection attempts (default: 3)
            retry_delay: Delay between retries in seconds (default: 0.5)
        
        Returns:
            True if connection successful, False otherwise
        """
        for attempt in range(max_retries):
            try:
                # Close any existing socket
                if self.socket:
                    try:
                        self.socket.close()
                    except Exception:
                        pass
                
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                self.socket.settimeout(self.timeout)
                self.socket.connect((self.host, self.port))
                self._connected = True
                print(f"Connected to RS485 gateway at {self.host}:{self.port}", flush=True)
                return True
            except Exception as e:
                if attempt < max_retries - 1:
                    print(f"Connection attempt {attempt + 1} failed: {e}, retrying...", flush=True)
                    time.sleep(retry_delay)
                else:
                    print(f"Failed to connect after {max_retries} attempts: {e}", flush=True)
                    self._connected = False
                    return False
        
        return False
    
    def disconnect(self):
        """Close the socket connection."""
        if self.socket:
            self.socket.close()
            self._connected = False
    
    def is_connected(self) -> bool:
        """Check if client is connected."""
        return self._connected
    
    def send_command(self, command: Union[List[int], bytes], 
                     response_delay: float = 0.1) -> Optional[bytes]:
        """
        Send a command and receive response.
        Thread-safe for concurrent access.
        
        Args:
            command: Command bytes as list of integers or bytes object
            response_delay: Delay before reading response in seconds (default: 0.1)
        
        Returns:
            Response bytes, or None if failed
        """
        if not self._connected:
            return None
        
        # Acquire lock to prevent concurrent socket access
        with self._lock:
            try:
                if isinstance(command, list):
                    command_bytes = bytes(command)
                else:
                    command_bytes = command
                
                self.socket.sendall(command_bytes)
                time.sleep(response_delay)
                response = self.socket.recv(1024)
                
                if len(response) == 0:
                    print(f"Warning: Received empty response", flush=True)
                    return None
                    
                return response
                
            except socket.timeout:
                print(f"Warning: Socket timeout while waiting for response", flush=True)
                return None
            except Exception as e:
                print(f"Error during communication: {e}", flush=True)
                return None
    
    @staticmethod
    def _calculate_crc16(data: bytes) -> int:
        """Calculate Modbus CRC16."""
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc
    
    def send_modbus_request(self, device_id: int, function_code: int, 
                           address: int, values: Union[int, List[int], None] = None,
                           count: int = 1, response_delay: float = 0.1) -> Optional[bytes]:
        """Send a Modbus RTU request with automatic CRC calculation."""
        if not self._connected:
            return None
        
        try:
            frame = bytearray()
            frame.append(device_id)
            frame.append(function_code)
            frame.extend(address.to_bytes(2, byteorder='big'))
            
            if function_code in [1, 3, 4]:  # Read operations
                if values is not None:
                    count = values if isinstance(values, int) else count
                frame.extend(count.to_bytes(2, byteorder='big'))
                
            elif function_code in [5, 6]:  # Write single
                if values is None:
                    raise ValueError("values parameter required for write operations")
                value = values if isinstance(values, int) else values[0]
                if function_code == 5:
                    value = 0xFF00 if value else 0x0000
                frame.extend(value.to_bytes(2, byteorder='big'))
                
            elif function_code in [15, 16]:  # Write multiple
                if values is None or not isinstance(values, list):
                    raise ValueError("values must be a list for write multiple operations")
                
                value_count = len(values)
                frame.extend(value_count.to_bytes(2, byteorder='big'))
                
                if function_code == 15:  # Write multiple coils
                    byte_count = (value_count + 7) // 8
                    frame.append(byte_count)
                    coil_bytes = bytearray((value_count + 7) // 8)
                    for i, val in enumerate(values):
                        if val:
                            coil_bytes[i // 8] |= (1 << (i % 8))
                    frame.extend(coil_bytes)
                    
                else:  # Write multiple registers (FC16)
                    byte_count = value_count * 2
                    frame.append(byte_count)
                    for val in values:
                        frame.extend(val.to_bytes(2, byteorder='big'))
            else:
                raise ValueError(f"Unsupported function code: {function_code}")
            
            # Calculate and append CRC16
            crc = self._calculate_crc16(bytes(frame))
            frame.append(crc & 0xFF)
            frame.append((crc >> 8) & 0xFF)
            
            return self.send_command(bytes(frame), response_delay=response_delay)
            
        except Exception as e:
            print(f"Error building Modbus request: {e}")
            return None
    
    @staticmethod
    def parse_modbus_response(response: bytes, function_code: int) -> dict:
        """Parse Modbus response and extract data."""
        if not response or len(response) < 5:
            return {'error': 'Invalid response length'}
        
        result = {
            'device_id': response[0],
            'function_code': response[1],
            'raw_hex': response.hex()
        }
        
        if response[1] & 0x80:
            result['error'] = True
            result['exception_code'] = response[2] if len(response) > 2 else None
            return result
        
        result['error'] = False
        
        if function_code in [1, 3, 4]:  # Read responses
            byte_count = response[2]
            data_bytes = response[3:3+byte_count]
            
            if function_code == 1:  # Coils
                result['byte_count'] = byte_count
                result['coils'] = []
                for byte in data_bytes:
                    for bit in range(8):
                        result['coils'].append(bool(byte & (1 << bit)))
            else:  # Registers
                result['byte_count'] = byte_count
                result['registers'] = []
                for i in range(0, len(data_bytes), 2):
                    if i + 1 < len(data_bytes):
                        value = (data_bytes[i] << 8) | data_bytes[i+1]
                        result['registers'].append(value)
        
        elif function_code in [5, 6, 15, 16]:  # Write responses
            result['address'] = (response[2] << 8) | response[3]
            result['value'] = (response[4] << 8) | response[5]
        
        return result
