import os
import shutil
from pathlib import Path

# Base directory
base_dir = Path(__file__).parent

# File mappings - {source: destination}
file_mappings = {
    # Core files
    'Client.py': 'src/core/client.py',
    'Command.py': 'src/core/command.py',
    'Thread.py': 'src/core/thread.py',
    'thread_safe.py': 'src/core/thread_safe.py',
    'network_manager.py': 'src/core/network_manager.py',
    'connections.py': 'src/core/connections.py',
    'video.py': 'src/core/video.py',
    
    # Models
    'Face.py': 'src/models/face.py',
    'PID.py': 'src/models/pid.py',
    
    # UI
    'ui_client.py': 'src/ui/main_window.py',
    'ui_face.py': 'src/ui/dialogs/face_dialog.py',
    'ui_led.py': 'src/ui/dialogs/led_dialog.py',
    'Calibration.py': 'src/ui/dialogs/calibration_dialog.py',
    
    # Utils
    'camera_recording.py': 'src/utils/camera_recorder.py',
    'logging_config.py': 'src/utils/logging_config.py',
    'utils.py': 'src/utils/utils.py',
    'exceptions.py': 'src/utils/exceptions.py',
    
    # Config
    'IP.txt': 'src/config/ip_config.txt',
    'point.txt': 'src/config/calibration_points.txt',
    
    # Tests
    'test_enhanced_network.py': 'tests/test_network.py',
    'test_network_manager.py': 'tests/test_network_manager.py',
    'verify_network.py': 'tests/verify_network.py',
    'run_debug.py': 'tests/run_debug.py',
}

def move_files():
    """Move files to their new locations."""
    for src, dst in file_mappings.items():
        src_path = base_dir / src
        dst_path = base_dir / dst
        
        # Create destination directory if it doesn't exist
        dst_path.parent.mkdir(parents=True, exist_ok=True)
        
        if src_path.exists():
            shutil.move(str(src_path), str(dst_path))
            print(f"Moved {src} to {dst}")
        else:
            print(f"Warning: Source file not found: {src}")

def create_init_files():
    """Create __init__.py files in all Python packages."""
    for root, dirs, _ in os.walk(base_dir / 'src'):
        if '__pycache__' in dirs:
            dirs.remove('__pycache__')
        for d in dirs:
            init_file = Path(root) / d / '__init__.py'
            if not init_file.exists():
                init_file.touch()
                print(f"Created {init_file}")

if __name__ == '__main__':
    print("Starting file organization...")
    move_files()
    create_init_files()
    print("File organization complete.")
