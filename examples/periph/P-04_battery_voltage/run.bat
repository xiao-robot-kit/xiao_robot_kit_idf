@echo off
rem =====================================================================
rem  P-04_battery_voltage -- Build & Flash (ESP-IDF v6.1)
rem
rem  WHAT THIS SCRIPT DOES:
rem    1. Checks required dependencies (ESP-IDF SDK + Python env)
rem    2. Builds the firmware by default (build-only mode)
rem    3. With a COMx argument: build + flash + open serial monitor
rem
rem  USAGE:
rem    run.bat          -> build only, no flash
rem    run.bat COM5     -> build, flash and monitor on COM5
rem
rem  DEPENDENCIES (checked at startup, explicit error if missing):
rem    - ESP-IDF SDK  : %IDF_PATH%\export.bat  (ESP-IDF v6.1)
rem    - Python env   : %IDF_PYTHON_ENV_PATH%  (idf6.1 virtualenv)
rem
rem  QEMU simulation: use run_qemu.bat instead (not this script).
rem =====================================================================
setlocal
if not defined IDF_PATH set IDF_PATH=C:\esp\v6.1\esp-idf
if not defined IDF_PYTHON_ENV_PATH set IDF_PYTHON_ENV_PATH=C:\Espressif\python_env\idf6.1_py3.11_env
set MSYSTEM=



rem ---- dependency check: ESP-IDF SDK ----
if not exist "%IDF_PATH%\export.bat" (
    echo [ERR] ESP-IDF SDK not found: %IDF_PATH%\export.bat
    echo       Install ESP-IDF v6.1, or fix IDF_PATH at the top of this script.
    pause
    exit /b 1
)
rem ---- dependency check: Python env ----
if not exist "%IDF_PYTHON_ENV_PATH%\Scripts\python.exe" (
    echo [ERR] IDF Python env not found: %IDF_PYTHON_ENV_PATH%
    echo       Create it via ESP-IDF installer, or fix IDF_PYTHON_ENV_PATH.
    pause
    exit /b 1
)

call "%IDF_PATH%\export.bat" >nul
if errorlevel 1 (
    echo [ERR] Failed to activate ESP-IDF environment.
    pause
    exit /b 1
)

if "%~1"=="" (
    echo [INFO] Build only (no port given). To flash+monitor: run.bat COMx
    idf.py build
) else (
    echo [INFO] Build + flash + monitor on port %~1
    idf.py -p %~1 flash monitor
)
endlocal
if errorlevel 1 pause
