"""Test script for the enhanced network implementation."""

import time
import unittest
from unittest.mock import MagicMock, patch, call

# Add parent directory to path for imports
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from Client.network_manager_v2 import NetworkManager
from Client.connections import SocketConnection, DummyConnection
from Client.interfaces import ConnectionState
from Client.exceptions import ConnectionError, CommandError


class TestNetworkManager(unittest.TestCase):
    """Test cases for NetworkManager."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.manager = NetworkManager(connection_class=DummyConnection)
    
    def test_initial_state(self):
        """Test initial state of NetworkManager."""
        self.assertEqual(self.manager.state, ConnectionState.DISNECTED)
        self.assertFalse(self.manager.is_connected)
    
    def test_connect_success(self):
        """Test successful connection."""
        self.manager.connect("127.0.0.1")
        self.assertEqual(self.manager.state, ConnectionState.CONNECTED)
        self.assertTrue(self.manager.is_connected)
    
    def test_disconnect(self):
        """Test disconnection."""
        self.manager.connect("127.0.0.1")
        self.manager.disconnect()
        self.assertEqual(self.manager.state, ConnectionState.DISCONNECTED)
    
    def test_send_command(self):
        """Test sending a command."""
        self.manager.connect("127.0.0.1")
        response = self.manager.send_command("TEST")
        self.assertTrue(response.success)
    
    def test_video_stream(self):
        """Test video streaming."""
        self.manager.connect("127.0.0.1")
        frame = self.manager.get_video_frame(timeout=1.0)
        self.assertIsNotNone(frame)
    
    def test_context_manager(self):
        """Test context manager usage."""
        with NetworkManager(connection_class=DummyConnection) as manager:
            manager.connect("127.0.0.1")
            self.assertEqual(manager.state, ConnectionState.CONNECTED)
        self.assertEqual(manager.state, ConnectionState.DISCONNECTED)


class TestSocketConnection(unittest.TestCase):
    """Test cases for SocketConnection."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.conn = DummyConnection()
    
    def test_connect_disconnect(self):
        """Test connection and disconnection."""
        self.conn.connect("127.0.0.1", 5002)
        self.assertEqual(self.conn.state, ConnectionState.CONNECTED)
        self.conn.disconnect()
        self.assertEqual(self.conn.state, ConnectionState.DISCONNECTED)
    
    def test_send_receive(self):
        """Test sending and receiving data."""
        self.conn.connect("127.0.0.1", 5002)
        self.conn.send(b"TEST")
        response = self.conn.receive(4)
        self.assertEqual(response, b"TEST")
    
    def test_context_manager(self):
        """Test context manager usage."""
        with DummyConnection() as conn:
            conn.connect("127.0.0.1", 5002)
            self.assertEqual(conn.state, ConnectionState.CONNECTED)
        self.assertEqual(conn.state, ConnectionState.DISCONNECTED)


if __name__ == "__main__":
    unittest.main()
