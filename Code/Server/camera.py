# -*- coding: utf-8 -*-
"""
Camera module for Freenove Big Hexapod Robot Kit

This module handles camera operations including video streaming, image capture,
and video recording using the Raspberry Pi Camera Module 2.
"""

import time
import os
import threading
from typing import Optional, Tuple, Dict, Any
from picamera2 import Picamera2, Preview
from picamera2.encoders import H264Encoder, JpegEncoder, MJPEGEncoder
from picamera2.outputs import FileOutput, CircularOutput
from libcamera import Transform, controls
from threading import Condition, Lock
import io
import cv2
import numpy as np


class StreamingOutput(io.BufferedIOBase):
    """
    Thread-safe streaming output for camera frames.
    
    Provides synchronized access to camera frames for multiple consumers.
    """
    
    def __init__(self, max_frames: int = 5):
        """
        Initialize streaming output.
        
        Args:
            max_frames (int): Maximum number of frames to buffer
        """
        self.frame = None
        self.frame_count = 0
        self.max_frames = max_frames
        self.condition = Condition()
        self._lock = Lock()

    def write(self, buf: bytes) -> int:
        """
        Write a buffer to the frame and notify waiting threads.
        
        Args:
            buf (bytes): Frame data buffer
            
        Returns:
            int: Number of bytes written
        """
        with self._lock:
            self.frame = buf
            self.frame_count += 1
            
        with self.condition:
            self.condition.notify_all()
        
        return len(buf)
    
    def get_frame(self) -> Optional[bytes]:
        """
        Get the current frame.
        
        Returns:
            bytes: Current frame data or None if not available
        """
        with self._lock:
            return self.frame
    
    def wait_for_frame(self, timeout: float = 1.0) -> Optional[bytes]:
        """
        Wait for a new frame with timeout.
        
        Args:
            timeout (float): Timeout in seconds
            
        Returns:
            bytes: New frame data or None if timeout
        """
        with self.condition:
            if self.condition.wait(timeout=timeout):
                return self.get_frame()
        return None


class Camera:
    """
    Camera class for Raspberry Pi Camera Module 2.
    
    Provides comprehensive camera functionality including streaming,
    recording, image capture, and various camera controls.
    """
    
    # Default configurations
    DEFAULT_PREVIEW_SIZE = (640, 480)
    DEFAULT_STREAM_SIZE = (400, 300)
    DEFAULT_RECORD_SIZE = (1920, 1080)
    DEFAULT_FPS = 30
    
    # Supported formats
    SUPPORTED_FORMATS = {
        'jpeg': JpegEncoder,
        'h264': H264Encoder,
        'mjpeg': MJPEGEncoder
    }
    
    def __init__(self, 
                 preview_size: Tuple[int, int] = DEFAULT_PREVIEW_SIZE,
                 stream_size: Tuple[int, int] = DEFAULT_STREAM_SIZE,
                 record_size: Tuple[int, int] = DEFAULT_RECORD_SIZE,
                 fps: int = DEFAULT_FPS,
                 hflip: bool = False,
                 vflip: bool = False,
                 rotation: int = 0):
        """
        Initialize the camera with specified configuration.
        
        Args:
            preview_size (tuple): Preview resolution (width, height)
            stream_size (tuple): Streaming resolution (width, height)
            record_size (tuple): Recording resolution (width, height)
            fps (int): Frames per second
            hflip (bool): Horizontal flip
            vflip (bool): Vertical flip
            rotation (int): Rotation angle (0, 90, 180, 270)
        """
        self.preview_size = preview_size
        self.stream_size = stream_size
        self.record_size = record_size
        self.fps = fps
        self.rotation = rotation
        
        # Camera state
        self.is_initialized = False
        self.is_streaming = False
        self.is_recording = False
        self.is_preview_active = False
        
        # Camera objects
        self.camera = None
        self.streaming_output = None
        self.current_config = None
        
        # Threading
        self._lock = Lock()
        self._stream_thread = None
        
        # Initialize camera
        self._initialize_camera(hflip, vflip)
    
    def _initialize_camera(self, hflip: bool, vflip: bool) -> bool:
        """
        Initialize the camera hardware.
        
        Args:
            hflip (bool): Horizontal flip
            vflip (bool): Vertical flip
            
        Returns:
            bool: True if initialization successful
        """
        try:
            self.camera = Picamera2()
            
            # Create transform for image flipping and rotation
            self.transform = Transform(
                hflip=1 if hflip else 0,
                vflip=1 if vflip else 0,
                rotation=self.rotation
            )
            
            # Create configurations
            self.preview_config = self.camera.create_preview_configuration(
                main={"size": self.preview_size},
                transform=self.transform
            )
            
            self.stream_config = self.camera.create_video_configuration(
                main={"size": self.stream_size},
                transform=self.transform
            )
            
            self.record_config = self.camera.create_video_configuration(
                main={"size": self.record_size},
                transform=self.transform
            )
            
            # Initialize streaming output
            self.streaming_output = StreamingOutput()
            
            self.is_initialized = True
            print(f"Camera initialized: {self.preview_size} preview, {self.stream_size} stream")
            return True
            
        except Exception as e:
            print(f"Failed to initialize camera: {e}")
            self.is_initialized = False
            return False
    
    def start_preview(self, backend: str = "QTGL") -> bool:
        """
        Start camera preview.
        
        Args:
            backend (str): Preview backend ("QTGL", "DRM", "NULL")
            
        Returns:
            bool: True if successful
        """
        if not self.is_initialized:
            return False
        
        try:
            with self._lock:
                if not self.is_preview_active:
                    self.camera.configure(self.preview_config)
                    self.camera.start_preview(Preview.QTGL if backend == "QTGL" else Preview.DRM)
                    self.camera.start()
                    self.is_preview_active = True
                    self.current_config = "preview"
                    print("Camera preview started")
            
            return True
            
        except Exception as e:
            print(f"Error starting preview: {e}")
            return False
    
    def stop_preview(self) -> bool:
        """
        Stop camera preview.
        
        Returns:
            bool: True if successful
        """
        if not self.is_initialized:
            return False
        
        try:
            with self._lock:
                if self.is_preview_active:
                    self.camera.stop_preview()
                    self.camera.stop()
                    self.is_preview_active = False
                    self.current_config = None
                    print("Camera preview stopped")
            
            return True
            
        except Exception as e:
            print(f"Error stopping preview: {e}")
            return False
    
    def start_stream(self, format_type: str = "jpeg") -> bool:
        """
        Start video streaming.
        
        Args:
            format_type (str): Stream format ("jpeg", "h264", "mjpeg")
            
        Returns:
            bool: True if successful
        """
        if not self.is_initialized:
            return False
        
        if format_type not in self.SUPPORTED_FORMATS:
            print(f"Unsupported format: {format_type}")
            return False
        
        try:
            with self._lock:
                if not self.is_streaming:
                    # Stop current operations
                    if self.is_preview_active:
                        self.stop_preview()
                    if self.is_recording:
                        self.stop_recording()
                    
                    # Configure for streaming
                    self.camera.configure(self.stream_config)
                    
                    # Create encoder and output
                    encoder_class = self.SUPPORTED_FORMATS[format_type]
                    encoder = encoder_class()
                    output = FileOutput(self.streaming_output)
                    
                    # Start streaming
                    self.camera.start_recording(encoder, output)
                    self.is_streaming = True
                    self.current_config = "stream"
                    print(f"Video streaming started ({format_type.upper()})")
            
            return True
            
        except Exception as e:
            print(f"Error starting stream: {e}")
            return False
    
    def stop_stream(self) -> bool:
        """
        Stop video streaming.
        
        Returns:
            bool: True if successful
        """
        if not self.is_initialized:
            return False
        
        try:
            with self._lock:
                if self.is_streaming:
                    self.camera.stop_recording()
                    self.is_streaming = False
                    self.current_config = None
                    print("Video streaming stopped")
            
            return True
            
        except Exception as e:
            print(f"Error stopping stream: {e}")
            return False
    
    def get_frame(self, timeout: float = 1.0) -> Optional[bytes]:
        """
        Get current frame from stream.
        
        Args:
            timeout (float): Timeout in seconds
            
        Returns:
            bytes: Frame data or None if timeout
        """
        if not self.is_streaming or not self.streaming_output:
            return None
        
        return self.streaming_output.wait_for_frame(timeout)
    
    def capture_image(self, filename: str, quality: int = 95) -> Optional[Dict[str, Any]]:
        """
        Capture and save a single image.
        
        Args:
            filename (str): Output filename
            quality (int): JPEG quality (1-100)
            
        Returns:
            dict: Image metadata or None if failed
        """
        if not self.is_initialized:
            return None
        
        try:
            with self._lock:
                # Configure for still capture
                still_config = self.camera.create_still_configuration(
                    main={"size": self.record_size},
                    transform=self.transform
                )
                self.camera.configure(still_config)
                self.camera.start()
                
                # Capture image
                metadata = self.camera.capture_file(filename, quality=quality)
                
                # Restore previous configuration
                if self.current_config == "preview":
                    self.camera.configure(self.preview_config)
                    self.camera.start_preview(Preview.QTGL)
                elif self.current_config == "stream":
                    self.camera.configure(self.stream_config)
                
                print(f"Image captured: {filename}")
                return metadata
                
        except Exception as e:
            print(f"Error capturing image: {e}")
            return None
    
    def start_recording(self, filename: str, format_type: str = "h264") -> bool:
        """
        Start video recording.
        
        Args:
            filename (str): Output filename
            format_type (str): Video format ("h264", "mjpeg")
            
        Returns:
            bool: True if successful
        """
        if not self.is_initialized:
            return False
        
        if format_type not in ["h264", "mjpeg"]:
            print(f"Unsupported recording format: {format_type}")
            return False
        
        try:
            with self._lock:
                if not self.is_recording:
                    # Stop current operations
                    if self.is_preview_active:
                        self.stop_preview()
                    if self.is_streaming:
                        self.stop_stream()
                    
                    # Configure for recording
                    self.camera.configure(self.record_config)
                    
                    # Create encoder and output
                    encoder_class = self.SUPPORTED_FORMATS[format_type]
                    encoder = encoder_class()
                    output = FileOutput(filename)
                    
                    # Start recording
                    self.camera.start_recording(encoder, output)
                    self.is_recording = True
                    self.current_config = "record"
                    print(f"Video recording started: {filename}")
            
            return True
            
        except Exception as e:
            print(f"Error starting recording: {e}")
            return False
    
    def stop_recording(self) -> bool:
        """
        Stop video recording.
        
        Returns:
            bool: True if successful
        """
        if not self.is_initialized:
            return False
        
        try:
            with self._lock:
                if self.is_recording:
                    self.camera.stop_recording()
                    self.is_recording = False
                    self.current_config = None
                    print("Video recording stopped")
            
            return True
            
        except Exception as e:
            print(f"Error stopping recording: {e}")
            return False
    
    def record_video(self, filename: str, duration: float, format_type: str = "h264") -> bool:
        """
        Record a video for specified duration.
        
        Args:
            filename (str): Output filename
            duration (float): Recording duration in seconds
            format_type (str): Video format
            
        Returns:
            bool: True if successful
        """
        if not self.start_recording(filename, format_type):
            return False
        
        time.sleep(duration)
        return self.stop_recording()
    
    def set_camera_controls(self, **controls_dict) -> bool:
        """
        Set camera controls (exposure, gain, etc.).
        
        Args:
            **controls_dict: Camera control parameters
            
        Returns:
            bool: True if successful
        """
        if not self.is_initialized:
            return False
        
        try:
            self.camera.set_controls(controls_dict)
            return True
        except Exception as e:
            print(f"Error setting camera controls: {e}")
            return False
    
    def get_camera_info(self) -> Dict[str, Any]:
        """
        Get camera information and status.
        
        Returns:
            dict: Camera information
        """
        info = {
            'initialized': self.is_initialized,
            'preview_active': self.is_preview_active,
            'streaming': self.is_streaming,
            'recording': self.is_recording,
            'current_config': self.current_config,
            'preview_size': self.preview_size,
            'stream_size': self.stream_size,
            'record_size': self.record_size,
            'fps': self.fps,
            'rotation': self.rotation
        }
        
        if self.is_initialized:
            try:
                info['camera_properties'] = self.camera.camera_properties
            except:
                pass
        
        return info
    
    def get_frame_as_cv2(self, timeout: float = 1.0) -> Optional[np.ndarray]:
        """
        Get frame as OpenCV numpy array.
        
        Args:
            timeout (float): Timeout in seconds
            
        Returns:
            np.ndarray: OpenCV image or None if failed
        """
        frame_data = self.get_frame(timeout)
        if frame_data is None:
            return None
        
        try:
            # Convert bytes to numpy array
            nparr = np.frombuffer(frame_data, np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            return img
        except Exception as e:
            print(f"Error converting frame to OpenCV: {e}")
            return None

    def close(self) -> None:
        """Close the camera and release resources."""
        try:
            with self._lock:
                if self.is_preview_active:
                    self.stop_preview()
                if self.is_streaming:
                    self.stop_stream()
                if self.is_recording:
                    self.stop_recording()
                
                if self.camera:
                    self.camera.close()
                    self.camera = None
                
                self.is_initialized = False
                print("Camera closed")
                
        except Exception as e:
            print(f"Error closing camera: {e}")
    
    def __enter__(self):
        """Context manager entry point."""
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        """Context manager exit point."""
        self.close()


def test_camera():
    """Test function to demonstrate camera functionality."""
    print("Testing Camera Module...")
    print("=" * 40)
    
    with Camera() as camera:
        # Test camera info
        print("\n1. Camera Information:")
        info = camera.get_camera_info()
        for key, value in info.items():
            print(f"  {key}: {value}")
        
        # Test preview
        print("\n2. Testing Preview (5 seconds)...")
        if camera.start_preview():
            time.sleep(5)
            camera.stop_preview()
        
        # Test image capture
        print("\n3. Testing Image Capture...")
        metadata = camera.capture_image("test_image.jpg")
        if metadata:
            print(f"  Image captured successfully")
            print(f"  Metadata: {metadata}")
        
        # Test video streaming
        print("\n4. Testing Video Streaming (3 seconds)...")
        if camera.start_stream():
            time.sleep(3)
            camera.stop_stream()
        
        # Test video recording
        print("\n5. Testing Video Recording (2 seconds)...")
        if camera.record_video("test_video.h264", 2):
            print("  Video recorded successfully")
        
        # Test camera controls
        print("\n6. Testing Camera Controls...")
        camera.set_camera_controls(AeEnable=True, AwbEnable=True)
    
    print("\n✓ Camera test completed successfully")


if __name__ == '__main__':
    print("Camera Module for Hexapod Robot")
    print("=" * 40)
    
    # Run test if executed directly
    test_camera()