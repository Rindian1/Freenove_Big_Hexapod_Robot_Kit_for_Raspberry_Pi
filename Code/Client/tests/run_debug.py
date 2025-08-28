#!/usr/bin/env python3
"""Debug script for running the Hexapod Robot client with debug output."""

import sys
import os
import logging

# Set up logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout)
    ]
)

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from PyQt5.QtWidgets import QApplication
    from Client.main import load_styles, MyWindow
    
    # Debug info
    print("Starting Hexapod Robot Client...")
    print(f"Python version: {sys.version}")
    print(f"Current directory: {os.getcwd()}")
    print(f"Python path: {sys.path}")
    
    # Create and run the application
    app = QApplication(sys.argv)
    print("QApplication created")
    
    # Load styles
    try:
        load_styles(app)
        print("Styles loaded successfully")
    except Exception as e:
        print(f"Error loading styles: {e}")
    
    # Create main window
    print("Creating main window...")
    window = MyWindow()
    print("Main window created")
    
    # Show window
    print("Showing main window...")
    window.show()
    print("Main window shown")
    
    # Run application
    print("Starting application event loop...")
    sys.exit(app.exec_())
    
except Exception as e:
    print(f"Fatal error: {e}", file=sys.stderr)
    import traceback
    traceback.print_exc()
    sys.exit(1)
