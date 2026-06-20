@echo off
REM ============================================================
REM  Offline demo: synthetic glove data -> retarget -> panel
REM  No hardware, no serial writes. Double-click to run.
REM  Panel: http://127.0.0.1:8090/
REM ============================================================
setlocal
set KMP_DUPLICATE_LIB_OK=TRUE
cd /d "%~dp0src\teleop"

echo [1/3] Starting synthetic glove streamer (new window)...
start "fake_streamer" cmd /k conda run -n teleop --no-capture-output python tools\fake_streamer.py --mode wave

echo [2/3] Opening 3D view (7000) + tuning panel (8090) ...
"%SystemRoot%\System32\timeout.exe" /t 6 /nobreak >nul 2>&1
start "" http://127.0.0.1:7000/static/
start "" http://127.0.0.1:8090/

echo [3/3] Starting retargeting service (this window; Ctrl+C to stop)...
conda run -n teleop --no-capture-output python -m dexhand_teleop.main

echo.
echo Service stopped. You can close the fake_streamer window.
pause
