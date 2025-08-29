# Hexapod Robot Control

Control software for the Hexapod Robot, featuring a user-friendly interface for controlling the robot's movements, camera, and other features.

## Project Structure

```
Client/
├── src/                         # Source code
│   ├── __init__.py
│   ├── main.py                  # Application entry point
│   ├── config/                  # Configuration files
│   │   ├── __init__.py
│   │   ├── ip_config.txt        # IP configuration
│   │   ├── calibration_points.txt  # Calibration data
│   │   └── styles/              # Style sheets
│   ├── core/                    # Core functionality
│   │   ├── __init__.py
│   │   ├── client.py           # Network client
│   │   ├── command.py          # Command definitions
│   │   ├── connections.py      # Network connections
│   │   ├── network_manager.py  # Network management
│   │   ├── thread.py           # Thread utilities
│   │   ├── thread_safe.py      # Thread-safe utilities
│   │   └── video.py            # Video streaming
│   ├── models/                  # Data models
│   │   ├── __init__.py
│   │   ├── face.py            # Face recognition
│   │   └── pid.py             # PID controller
│   ├── ui/                      # User interface
│   │   ├── __init__.py
│   │   ├── main_window.py     # Main application window
│   │   └── dialogs/           # Dialog windows
│   │       ├── __init__.py
│   │       ├── calibration_dialog.py
│   │       ├── face_dialog.py
│   │       └── led_dialog.py
│   └── utils/                  # Utility functions
│       ├── __init__.py
│       ├── camera_recorder.py  # Camera functionality
│       ├── exceptions.py       # Custom exceptions
│       ├── logging_config.py   # Logging configuration
│       └── utils.py           # Utility functions
└── tests/                      # Unit and integration tests
    ├── __init__.py
    ├── test_network.py
    ├── test_network_manager.py
    └── verify_network.py
```

## Installation

1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd hexapod-robot-control/Client
   ```

2. Create a virtual environment (recommended):
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Usage

To start the application:

```bash
python src/main.py
```

## Development

### Running Tests

```bash
pytest tests/
```

### Code Style

This project uses `black` for code formatting and `flake8` for linting.

```bash
black src/
flake8 src/
```

