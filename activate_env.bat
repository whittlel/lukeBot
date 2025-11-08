@echo off
REM Activate virtual environment for LukeBot
REM Windows batch script

if exist .venv\Scripts\activate.bat (
    call .venv\Scripts\activate.bat
    echo Virtual environment activated!
    echo.
    echo You can now run:
    echo   python main.py
    echo   python scripts/test_camera.py
    echo   etc.
    echo.
    echo To deactivate, type: deactivate
) else (
    echo ERROR: Virtual environment not found!
    echo Please run: python -m venv .venv
    echo Then install dependencies: pip install -r requirements.txt
)

