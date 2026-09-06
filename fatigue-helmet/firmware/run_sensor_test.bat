@echo off
REM Sensor Test Automation Script (Windows)
REM Finds your ESP32-S3 port, compiles, uploads, and captures output

setlocal enabledelayedexpansion

echo.
echo ╔════════════════════════════════════════════════════════════╗
echo ║      IoT FATIGUE HELMET — SENSOR TEST AUTOMATION            ║
echo ╚════════════════════════════════════════════════════════════╝
echo.

REM Try to find ESP32 port automatically
echo Scanning for ESP32-S3 COM ports...
for /f "tokens=1" %%A in ('wmic logicaldisk get name ^| find ":"') do (
    set "drive=%%A"
)

REM Check for common COM ports
set COM_PORT=
for %%P in (COM3 COM4 COM5 COM6 COM7 COM8 COM9) do (
    echo Checking %%P...
    REM This is a simple check; production scripts use more robust detection
    if exist \\.\%%P (
        set COM_PORT=%%P
        echo Found: %%P
        goto found_port
    )
)

:found_port
if "%COM_PORT%"=="" (
    echo.
    echo ERROR: Could not auto-detect COM port. Please specify manually:
    echo   run_sensor_test.bat COM3
    echo.
    exit /b 1
)

if not "%1"=="" (
    set COM_PORT=%1
    echo Using specified port: %COM_PORT%
)

echo Using COM port: %COM_PORT%
echo.

REM Step 1: Compile
echo ═══════════════════════════════════════════════════════════
echo Step 1/3: COMPILING SENSOR TEST FIRMWARE
echo ═══════════════════════════════════════════════════════════
pio run -e sensor_test -t upload --upload-port %COM_PORT%
if errorlevel 1 (
    echo ERROR: Compilation or upload failed
    exit /b 1
)

echo.
echo ✓ Upload complete. Waiting 3 seconds for board to reset...
timeout /t 3 /nobreak

REM Step 2: Capture output
echo.
echo ═══════════════════════════════════════════════════════════
echo Step 2/3: RUNNING SENSOR TEST (~100 seconds)
echo ═══════════════════════════════════════════════════════════
echo.

set TIMESTAMP=%date:~-4%%date:~-10,2%%date:~-7,2%_%time:~0,2%%time:~3,2%%time:~6,2%
set TIMESTAMP=%TIMESTAMP: =0%
set OUTPUT_FILE=sensor_test_result_%TIMESTAMP%.txt

echo Capturing output to: %OUTPUT_FILE%
echo.
pio device monitor -b 921600 -p %COM_PORT% > "%OUTPUT_FILE%" 2>&1
timeout /t 5 /nobreak

REM Step 3: Analyze
echo.
echo ═══════════════════════════════════════════════════════════
echo Step 3/3: ANALYZING RESULTS
echo ═══════════════════════════════════════════════════════════
echo.

if exist "%OUTPUT_FILE%" (
    echo Parsing results...
    python analyze_sensor_test.py "%OUTPUT_FILE%"
    echo.
    echo Full output saved to: %OUTPUT_FILE%
    echo.
    REM Open result file for inspection
    echo Opening test result in default text editor...
    start "" "%OUTPUT_FILE%"
) else (
    echo ERROR: Output file not created
    exit /b 1
)

echo.
echo ╔════════════════════════════════════════════════════════════╗
echo ║                  TEST COMPLETE                             ║
echo ╚════════════════════════════════════════════════════════════╝
echo.
pause
