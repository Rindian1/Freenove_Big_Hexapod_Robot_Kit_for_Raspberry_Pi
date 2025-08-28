#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Hexapod Robot Control - Main Entry Point
"""

import os
import sys
from pathlib import Path

# Add the src directory to the Python path
sys.path.insert(0, str(Path(__file__).parent))

def main():
    """Main application entry point."""
    try:
        from PyQt5.QtWidgets import QApplication
        from src.ui.main_window import MainWindow
        from src.utils.logging_config import setup_logging
        
        # Set up logging
        setup_logging()
        
        # Create application
        app = QApplication(sys.argv)
        
        # Set application style
        apply_styles(app)
        
        # Create and show main window
        window = MainWindow()
        window.show()
        
        # Start event loop
        sys.exit(app.exec_())
        
    except ImportError as e:
        print(f"Error: {e}")
        print("Please install the required dependencies using:")
        print("pip install -r requirements.txt")
        sys.exit(1)

def apply_styles(app):
    """Apply styles to the application."""
    try:
        styles_dir = Path(__file__).parent / 'config' / 'styles'
        style_file = styles_dir / 'styles.qss'
        
        if style_file.exists():
            with open(style_file, 'r') as f:
                app.setStyleSheet(f.read())
    except Exception as e:
        print(f"Warning: Could not load styles: {e}")

if __name__ == "__main__":
    main()
