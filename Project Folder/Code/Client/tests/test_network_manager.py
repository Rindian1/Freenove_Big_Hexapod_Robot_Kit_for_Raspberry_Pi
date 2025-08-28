"""Unit tests for the NetworkManager class."""

import unittest
import socket
import threading
import time
import struct
import sys
import os

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from unittest.mock import MagicMock, patch, call

# Local imports
from .network_manager import NetworkManager, ConnectionState
from .exceptions import ConnectionError, TimeoutError
from .client_v2 import Client


class MockSocket:
    """Mock socket for testing network operations."""
    
    def __init__(self, *args, **kwargs):
        self.connected = False
        self.recv_data = []
        self.sent_data = []
        self.timeout = None
        self.blocking = True
        self.raise_on_connect = False
        self.raise_on_recv = False
        self.raise_on_send = False
    
    def settimeout(self, timeout):
        self.timeout = timeout
    
    def setblocking(self, flag):
        self.blocking = flag
    
    def connect(self, address):
        if self.raise_on_connect:
            raise socket.error("Connection failed")
        self.connected = True
    
    def sendall(self, data):
        if self.raise_on_send:
            raise socket.error("Send failed")
        self.sent_data.append(data)
    
    def recv(self, bufsize):
        if self.raise_on_recv:
            raise socket.error("Receive failed")
        if not self.recv_data:
            return b''
        return self.recv_data.pop(0)
    
    def close(self):
        self.connected = False
    
    def shutdown(self, how):
        pass


class TestNetworkManager(unittest.TestCase):
    """Test cases for NetworkManager class."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.client = MagicMock(spec=Client)
        self.manager = NetworkManager(self.client)
        
        # Patch socket creation
        self.socket_patcher = patch('socket.socket')
        self.mock_socket = self.socket_patcher.start()
        
        # Create mock sockets for video and instruction channels
        self.video_socket = MockSocket()
        self.instruction_socket = MockSocket()
        self.mock_socket.side_effect = [self.instruction_socket, self.video_socket]
        
        # Default test values
        self.test_ip = "192.168.1.100"
        self.test_port = 5002
        self.test_video_port = 8002
    
    def tearDown(self):
        """Clean up after tests."""
        self.manager.disconnect()
        self.socket_patcher.stop()
    
    def test_initial_state(self):
        """Test initial state of NetworkManager."""
        self.assertEqual(self.manager.state, ConnectionState.DISCONNECTED)
        self.assertIsNone(self.manager.ip)
        self.assertIsNone(self.manager.port)
        self.assertIsNone(self.manager.video_port)
    
    def test_connect_success(self):
        """Test successful connection to the robot."""
        # Test
        self.manager.connect(self.test_ip, self.test_port, self.test_video_port)
        
        # Assert
        self.assertEqual(self.manager.state, ConnectionState.CONNECTED)
        self.assertEqual(self.manager.ip, self.test_ip)
        self.assertEqual(self.manager.port, self.test_port)
        self.assertEqual(self.manager.video_port, self.test_video_port)
        
        # Verify sockets were created and connected
        self.mock_socket.assert_has_calls([
            call(socket.AF_INET, socket.SOCK_STREAM),
            call(socket.AF_INET, socket.SOCK_STREAM)
        ])
        
        # Verify client was initialized
        self.client.turn_on_client.assert_called_once_with(self.test_ip)
        
        # Verify threads were started
        self.assertIsNotNone(self.manager._video_thread)
        self.assertIsNotNone(self.manager._instruction_thread)
    
    def test_connect_failure(self):
        """Test connection failure handling."""
        # Setup
        self.instruction_socket.raise_on_connect = True
        
        # Test & Assert
        with self.assertRaises(ConnectionError):
            self.manager.connect(self.test_ip, self.test_port, self.test_video_port)
        
        self.assertEqual(self.manager.state, ConnectionState.DISCONNECTED)
    
    def test_disconnect(self):
        """Test disconnection."""
        # Setup
        self.manager.connect(self.test_ip, self.test_port, self.test_video_port)
        
        # Test
        self.manager.disconnect()
        
        # Assert
        self.assertEqual(self.manager.state, ConnectionState.DISCONNECTED)
        self.assertFalse(self.video_socket.connected)
        self.assertFalse(self.instruction_socket.connected)
        
        # Verify client was turned off
        self.client.turn_off_client.assert_called_once()
    
    def test_send_command_success(self):
        """Test sending a command successfully."""
        # Setup
        self.manager.connect(self.test_ip, self.test_port, self.test_video_port)
        test_command = "TEST_COMMAND\n"
        
        # Test
        self.manager.send_command(test_command)
        
        # Assert
        self.assertEqual(len(self.instruction_socket.sent_data), 1)
        self.assertEqual(self.instruction_socket.sent_data[0], test_command.encode('utf-8'))
    
    def test_send_command_not_connected(self):
        """Test sending a command when not connected."""
        test_command = "TEST_COMMAND\n"
        
        # Test & Assert
        with self.assertRaises(ConnectionError):
            self.manager.send_command(test_command)
    
    def test_receive_command_success(self):
        """Test receiving a command successfully."""
        # Setup
        self.manager.connect(self.test_ip, self.test_port, self.test_video_port)
        test_response = b"RESPONSE\n"
        self.instruction_socket.recv_data = [test_response]
        
        # Test
        response = self.manager.receive_command(timeout=1.0)
        
        # Assert
        self.assertEqual(response, test_response.decode('utf-8'))
    
    def test_receive_command_timeout(self):
        """Test receive command timeout."""
        # Setup
        self.manager.connect(self.test_ip, self.test_port, self.test_video_port)
        self.instruction_socket.recv_data = []  # No data to receive
        
        # Test & Assert
        with self.assertRaises(TimeoutError):
            self.manager.receive_command(timeout=0.1)
    
    def test_video_thread_processing(self):
        """Test video frame processing in the video thread."""
        # Setup
        self.manager.connect(self.test_ip, self.test_port, self.test_video_port)
        
        # Create a test frame (simplified)
        frame_header = struct.pack('<L', 8)  # Frame length
        frame_data = b'\xff\xd8' + b'\x00' * 6 + b'\xff\xd9'  # Minimal JPEG
        self.video_socket.recv_data = [frame_header + frame_data]
        
        # Let the thread run for a bit
        time.sleep(0.1)
        
        # Test
        frame = self.manager.get_video_frame()
        
        # Assert
        self.assertIsNotNone(frame)
        self.assertEqual(frame[:2], b'\xff\xd8')  # JPEG start marker
        self.assertEqual(frame[-2:], b'\xff\xd9')  # JPEG end marker
    
    def test_instruction_thread_processing(self):
        """Test instruction processing in the instruction thread."""
        # Setup
        self.manager.connect(self.test_ip, self.test_port, self.test_video_port)
        test_instruction = b"INSTRUCTION_DATA\n"
        self.instruction_socket.recv_data = [test_instruction]
        
        # Let the thread run for a bit
        time.sleep(0.1)
        
        # Assert
        self.client.process_instruction.assert_called_once_with(test_instruction)
    
    def test_reconnect_mechanism(self):
        """Test automatic reconnection on connection loss."""
        # Setup - initial successful connection
        self.manager.connect(self.test_ip, self.test_port, self.test_video_port)
        
        # Simulate connection drop
        self.instruction_socket.connected = False
        self.video_socket.connected = False
        
        # Queue up a failure and then success for the reconnection attempt
        self.instruction_socket.raise_on_connect = True
        self.manager._reconnect_attempts = 0  # Reset attempts
        
        # Let the reconnection logic run
        time.sleep(0.2)
        
        # Should be in RECONNECTING state after failure
        self.assertEqual(self.manager.state, ConnectionState.RECONNECTING)
        
        # Now allow reconnection to succeed
        self.instruction_socket.raise_on_connect = False
        
        # Let the reconnection complete
        time.sleep(0.2)
        
        # Should be connected again
        self.assertEqual(self.manager.state, ConnectionState.CONNECTED)


if __name__ == '__main__':
    unittest.main()
