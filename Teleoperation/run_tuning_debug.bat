@echo off
REM Wheel Actuator Tuning Debug Tool Launcher
REM This script launches the tuning debug tool with proper Python environment

echo ============================================================
echo   Wheel Actuator Tuning Debug Tool
echo ============================================================
echo.

REM Check if Python is available
python --version >nul 2>&1
if errorlevel 1 (
    echo ERROR: Python is not installed or not in PATH
    echo Please install Python 3.7 or later
    pause
    exit /b 1
)

REM Check if websockets module is installed
python -c "import websockets" >nul 2>&1
if errorlevel 1 (
    echo Installing required dependencies...
    pip install websockets
    if errorlevel 1 (
        echo ERROR: Failed to install websockets
        echo Please run: pip install websockets
        pause
        exit /b 1
    )
)

REM Run the tuning debug tool
echo Starting tuning debug tool...
echo.
python tuning_debug.py

pause

