@echo off
rem =====================================================================
rem  exp_motor_enc -- build + flash + monitor (real board)
rem
rem  Usage:
rem    run.bat            build only
rem    run.bat COM5       build, flash to COM5, open the serial monitor
rem
rem  Requires ESP-IDF: set IDF_PATH (or install to the default location).
rem =====================================================================
setlocal
cd /d "%~dp0"
set MSYSTEM=
if not defined IDF_PATH set IDF_PATH=C:\esp\v6.1\esp-idf
if not defined IDF_PYTHON_ENV_PATH set IDF_PYTHON_ENV_PATH=C:\Espressif\python_env\idf6.1_py3.11_env
call "%IDF_PATH%\export.bat" >nul
if errorlevel 1 (
    echo [ERR] Failed to activate ESP-IDF environment: %IDF_PATH%
    pause
    exit /b 1
)
if "%1"=="" (
    idf.py build
) else (
    idf.py -p %1 flash monitor
)
if errorlevel 1 (
    echo [ERR] build/flash failed.
    pause
    exit /b 1
)
pause
endlocal
