#!/bin/bash
# Activate virtual environment for LukeBot
# Linux/Mac shell script

if [ -f .venv/bin/activate ]; then
    source .venv/bin/activate
    echo "Virtual environment activated!"
    echo ""
    echo "You can now run:"
    echo "  python main.py"
    echo "  python scripts/test_camera.py"
    echo "  etc."
    echo ""
    echo "To deactivate, type: deactivate"
else
    echo "ERROR: Virtual environment not found!"
    echo "Please run: python3 -m venv .venv"
    echo "Then install dependencies: pip install -r requirements.txt"
fi

