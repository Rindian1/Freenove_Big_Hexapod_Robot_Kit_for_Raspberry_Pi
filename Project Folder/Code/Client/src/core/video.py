"""Video streaming implementation for Hexapod Robot."""

import struct
import threading
import time
from typing import Optional, Callable, Any

from .interfaces import IVideoStream, IConnection
from .exceptions import VideoError, ConnectionError
from .thread_safe import ThreadSafeValue
from .logging_config import get_logger

logger = get_logger(__name__)

class VideoStream(IVideoStream):
    """Handles video streaming from the robot."""
    
    def __init__(self, connection: IConnection, frame_callback: Optional[Callable[[bytes], None]] = None):
        """Initialize the video stream.
        
        Args:
            connection: The connection to use for video streaming
            frame_callback: Optional callback for processing frames
        """
        self._conn = connection
        self._frame_callback = frame_callback
        self._running = ThreadSafeValue(False, name="video_running")
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.RLock()
        self._current_frame = ThreadSafeValue(None, name="current_frame")
        self._frame_ready = threading.Event()
    
    @property
    def is_running(self) -> bool:
        """Check if the video stream is running."""
        return self._running.value
    
    def start(self) -> None:
        """Start the video stream."""
        with self._lock:
            if self._running.value:
                logger.warning("Video stream is already running")
                return
                
            if self._conn.state != ConnectionState.CONNECTED:
                raise ConnectionError("Not connected to video source")
                
            self._running.value = True
            self._frame_ready.clear()
            self._thread = threading.Thread(
                target=self._stream_loop,
                name="VideoStreamThread",
                daemon=True
            )
            self._thread.start()
            logger.info("Video stream started")
    
    def stop(self) -> None:
        """Stop the video stream."""
        with self._lock:
            if not self._running.value:
                return
                
            self._running.value = False
            self._frame_ready.set()  # Unblock any waiting gets
            
            if self._thread and self._thread.is_alive():
                self._thread.join(timeout=2.0)
                if self._thread.is_alive():
                    logger.warning("Video thread did not stop gracefully")
            
            self._thread = None
            logger.info("Video stream stopped")
    
    def get_frame(self, timeout: Optional[float] = None) -> Optional[bytes]:
        """Get the next video frame.
        
        Args:
            timeout: Maximum time to wait for a frame in seconds
            
        Returns:
            Optional[bytes]: The frame data or None if no frame is available
        """
        if not self._running.value:
            return None
            
        if self._frame_ready.wait(timeout=timeout):
            return self._current_frame.value
        return None
    
    def _stream_loop(self) -> None:
        """Main streaming loop."""
        buffer = bytearray()
        
        try:
            while self._running.value:
                try:
                    # Read frame header (4 bytes for length)
                    header = self._conn.receive(4, timeout=1.0)
                    if len(header) != 4:
                        logger.warning("Invalid frame header received")
                        continue
                    
                    # Get frame length from header
                    frame_length = struct.unpack('<L', header)[0]
                    if frame_length == 0 or frame_length > 10 * 1024 * 1024:  # 10MB max
                        logger.warning(f"Invalid frame length: {frame_length}")
                        continue
                    
                    # Read frame data
                    frame_data = bytearray()
                    remaining = frame_length
                    
                    while remaining > 0 and self._running.value:
                        chunk = self._conn.receive(min(4096, remaining), timeout=1.0)
                        if not chunk:
                            break
                        frame_data.extend(chunk)
                        remaining -= len(chunk)
                    
                    if len(frame_data) != frame_length:
                        logger.warning(f"Incomplete frame received: {len(frame_data)}/{frame_length} bytes")
                        continue
                    
                    # Update current frame
                    frame_bytes = bytes(frame_data)
                    self._current_frame.value = frame_bytes
                    self._frame_ready.set()
                    
                    # Notify callback if provided
                    if self._frame_callback:
                        try:
                            self._frame_callback(frame_bytes)
                        except Exception as e:
                            logger.error(f"Error in frame callback: {e}", exc_info=True)
                    
                except ConnectionError as e:
                    logger.error(f"Connection error in video stream: {e}")
                    self._running.value = False
                    break
                except Exception as e:
                    logger.error(f"Error in video stream: {e}", exc_info=True)
                    time.sleep(0.1)  # Prevent tight loop on errors
        
        except Exception as e:
            logger.critical(f"Fatal error in video stream: {e}", exc_info=True)
            self._running.value = False
            raise
        
        finally:
            self._running.value = False
            logger.info("Video stream loop ended")
    
    def __enter__(self):
        """Context manager entry."""
        self.start()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.stop()
    
    def __del__(self):
        """Ensure resources are cleaned up."""
        self.stop()
