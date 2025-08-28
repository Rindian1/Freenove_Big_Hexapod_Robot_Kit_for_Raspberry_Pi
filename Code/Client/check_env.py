"""Check Python environment and imports."""

import sys
import os

print("Python version:", sys.version)
print("\nPython path:")
for p in sys.path:
    print(f"  {p}")

print("\nCurrent working directory:", os.getcwd())
print("\nFiles in current directory:")
for f in os.listdir('.'):
    print(f"  {f}")

print("\nTrying to import required modules...")
try:
    import unittest
    from unittest.mock import MagicMock
    print("  - unittest: OK")
    print("  - unittest.mock: OK")
except ImportError as e:
    print(f"  - Error importing unittest: {e}")

try:
    from . import network_manager_v2
    print("  - network_manager_v2: OK")
except ImportError as e:
    print(f"  - Error importing network_manager_v2: {e}")

print("\nEnvironment check complete.")
