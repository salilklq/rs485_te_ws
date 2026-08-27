@echo off
REM ============================================================
REM  Real hardware: MANUS RawSkeleton -> retarget -> RS485 hand + panel
REM
REM  Prerequisites (this script does NOT do them for you):
REM    1) MANUS Core running, glove calibrated and streaming
REM    2) src\teleop\configs\drive.yml has the COM ports set (empty = dry-run)
REM    3) C++ streamer compiled (see README section 5)
REM  Panel: http://127.0.0.1:8090/
REM ============================================================
setlocal
set KMP_DUPLICATE_LIB_OK=TRUE
set EXE=%~dp0src\keypoint_streamer\Output\x64\Release\ManusKeypointStreamer_Windows.exe

if not exist "%EXE%" (
  echo [ERROR] Streamer not found: %EXE%
  echo         Build src\keypoint_streamer\ManusKeypointStreamer.sln ^(Release/x64^) first.
  pause
  exit /b 1
)

echo [0/3] Stopping any previous streamer / service instances (prevents COM-port contention)...
taskkill /F /IM ManusKeypointStreamer_Windows.exe >nul 2>&1
powershell -NoProfile -Command "Get-CimInstance Win32_Process | Where-Object { $_.Name -in @('python.exe','conda.exe') -and $_.CommandLine -match 'dexhand_teleop.main' } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue }" >nul 2>&1
"%SystemRoot%\System32\timeout.exe" /t 2 /nobreak >nul 2>&1

echo [1/3] Starting MANUS keypoint streamer (new window)...
start "ManusKeypointStreamer" cmd /k "%EXE%"

echo [2/3] Opening 3D view (7000) + tuning panel (8090) ...
"%SystemRoot%\System32\timeout.exe" /t 6 /nobreak >nul 2>&1
start "" http://127.0.0.1:7000/static/
start "" http://127.0.0.1:8090/

echo [3/3] Starting retarget + drive service (this window; Ctrl+C to stop, relaxes on exit)...
cd /d "%~dp0src\teleop"
conda run -n teleop --no-capture-output python -m dexhand_teleop.main

echo.
echo Service stopped (relax-on-exit attempted). You can close the streamer window.
pause
