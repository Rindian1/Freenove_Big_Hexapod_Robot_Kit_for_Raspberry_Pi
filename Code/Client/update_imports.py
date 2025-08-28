import os
import re
from pathlib import Path

# Base directory
base_dir = Path(__file__).parent

# Import mappings - {old_import: new_import}
import_mappings = {
    # Core imports
    'from Client import': 'from src.core.client import',
    'from Command import': 'from src.core.command import',
    'from Thread import': 'from src.core.thread import',
    'from thread_safe import': 'from src.core.thread_safe import',
    'from network_manager import': 'from src.core.network_manager import',
    'from connections import': 'from src.core.connections import',
    'from video import': 'from src.core.video import',
    
    # Models
    'from Face import': 'from src.models.face import',
    'from PID import': 'from src.models.pid import',
    
    # Utils
    'from camera_recording import': 'from src.utils.camera_recorder import',
    'from logging_config import': 'from src.utils.logging_config import',
    'from utils import': 'from src.utils.utils import',
    'from exceptions import': 'from src.utils.exceptions import',
    
    # UI (these might be relative imports within the UI package)
    'from ui_client import': 'from .main_window import',
    'from ui_face import': 'from .dialogs.face_dialog import',
    'from ui_led import': 'from .dialogs.led_dialog import',
    'from Calibration import': 'from .dialogs.calibration_dialog import',
}

def update_imports_in_file(file_path):
    """Update import statements in a single file."""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        updated_content = content
        for old_import, new_import in import_mappings.items():
            updated_content = updated_content.replace(old_import, new_import)
        
        if updated_content != content:
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(updated_content)
            print(f"Updated imports in {file_path}")
            
    except Exception as e:
        print(f"Error processing {file_path}: {e}")

def process_directory(directory):
    """Process all Python files in a directory recursively."""
    for root, _, files in os.walk(directory):
        # Skip __pycache__ directories
        if '__pycache__' in root:
            continue
            
        for file in files:
            if file.endswith('.py') and file != 'update_imports.py':
                file_path = os.path.join(root, file)
                update_imports_in_file(file_path)

if __name__ == '__main__':
    print("Updating imports...")
    process_directory(base_dir / 'src')
    process_directory(base_dir / 'tests')
    print("Finished updating imports.")
