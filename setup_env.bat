@echo off
REM Setup script for LukeBot virtual environment
REM Windows batch script

echo ========================================
echo LukeBot - Environment Setup
echo ========================================
echo.

REM Check if Python is available
python --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python not found!
    echo Please install Python 3.8 or higher.
    pause
    exit /b 1
)

echo Python found: 
python --version
echo.

REM Create virtual environment if it doesn't exist
if not exist .venv (
    echo Creating virtual environment...
    python -m venv .venv
    if errorlevel 1 (
        echo ERROR: Failed to create virtual environment!
        pause
        exit /b 1
    )
    echo Virtual environment created!
) else (
    echo Virtual environment already exists.
)
echo.

REM Activate virtual environment
echo Activating virtual environment...
call .venv\Scripts\activate.bat
if errorlevel 1 (
    echo ERROR: Failed to activate virtual environment!
    pause
    exit /b 1
)
echo.

REM Upgrade pip
echo Upgrading pip...
python -m pip install --upgrade pip
echo.

REM Install dependencies
echo Installing dependencies...
pip install -r requirements.txt
if errorlevel 1 (
    echo ERROR: Failed to install dependencies!
    pause
    exit /b 1
)
echo.

echo ========================================
echo Setup complete!
echo ========================================
echo.
echo The virtual environment is now active.
echo You can run:
echo   python main.py
echo   python scripts/test_camera.py
echo   etc.
echo.
echo To activate the environment later, run:
echo   activate_env.bat
echo.
pause

