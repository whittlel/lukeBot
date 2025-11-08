#!/bin/bash
# Setup script for LukeBot virtual environment
# Linux/Mac shell script

echo "========================================"
echo "LukeBot - Environment Setup"
echo "========================================"
echo ""

# Check if Python is available
if ! command -v python3 &> /dev/null; then
    echo "ERROR: Python3 not found!"
    echo "Please install Python 3.8 or higher."
    exit 1
fi

echo "Python found:"
python3 --version
echo ""

# Create virtual environment if it doesn't exist
if [ ! -d ".venv" ]; then
    echo "Creating virtual environment..."
    python3 -m venv .venv
    if [ $? -ne 0 ]; then
        echo "ERROR: Failed to create virtual environment!"
        exit 1
    fi
    echo "Virtual environment created!"
else
    echo "Virtual environment already exists."
fi
echo ""

# Activate virtual environment
echo "Activating virtual environment..."
source .venv/bin/activate
if [ $? -ne 0 ]; then
    echo "ERROR: Failed to activate virtual environment!"
    exit 1
fi
echo ""

# Upgrade pip
echo "Upgrading pip..."
python -m pip install --upgrade pip
echo ""

# Install dependencies
echo "Installing dependencies..."
pip install -r requirements.txt
if [ $? -ne 0 ]; then
    echo "ERROR: Failed to install dependencies!"
    exit 1
fi
echo ""

echo "========================================"
echo "Setup complete!"
echo "========================================"
echo ""
echo "The virtual environment is now active."
echo "You can run:"
echo "  python main.py"
echo "  python scripts/test_camera.py"
echo "  etc."
echo ""
echo "To activate the environment later, run:"
echo "  source activate_env.sh"
echo ""

