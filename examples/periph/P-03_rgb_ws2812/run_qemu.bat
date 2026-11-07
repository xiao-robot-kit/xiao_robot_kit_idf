@echo off
rem =====================================================================
rem  P-03_rgb_ws2812 -- QEMU variant build (SIM data source, compile-check only)
rem
rem  WHAT THIS SCRIPT DOES:
rem    1. Checks required dependencies (SDK / Python env)
rem    2. Builds the QEMU/SIM variant: -DP_SIM_LED=1 -DEXP_QEMU_UART=1
rem
rem  NOTE: booting this variant in QEMU and running text assertions is
rem  part of the private CI pipeline of the project (see repo docs).
rem  This public copy only builds the SIM variant so you can verify it
rem  compiles cleanly. For real-hardware runs use run.bat COMx.
rem
rem  DEPENDENCIES (checked at startup, explicit error if missing):
rem    - ESP-IDF SDK  : %IDF_PATH%\export.bat
rem    - Python env   : %IDF_PYTHON_ENV_PATH%
rem
rem  Real board build+flash: use run.bat COMx instead.
rem =====================================================================
setlocal
set MSYSTEM=
if not defined IDF_PATH set IDF_PATH=C:\esp\v6.1\esp-idf
if not defined IDF_PYTHON_ENV_PATH set IDF_PYTHON_ENV_PATH=C:\Espressif\python_env\idf6.1_py3.11_env

rem ---- dependency check: ESP-IDF SDK ----
if not exist "%IDF_PATH%\export.bat" (
    echo [ERR] ESP-IDF SDK not found: %IDF_PATH%\export.bat
    echo       Install ESP-IDF v6.1, or set IDF_PATH before running this script.
    pause
    exit /b 1
)
rem ---- dependency check: Python env ----
if not exist "%IDF_PYTHON_ENV_PATH%\Scripts\python.exe" (
    echo [ERR] IDF Python env not found: %IDF_PYTHON_ENV_PATH%
    echo       Create it via ESP-IDF installer, or set IDF_PYTHON_ENV_PATH.
    pause
    exit /b 1
)

call "%IDF_PATH%\export.bat" >nul
if errorlevel 1 (
    echo [ERR] Failed to activate ESP-IDF environment.
    pause
    exit /b 1
)

echo [INFO] build QEMU/SIM variant (-DP_SIM_LED=1 -DEXP_QEMU_UART=1)
idf.py -B build-qemu -DP_SIM_LED=1 -DEXP_QEMU_UART=1 build
if errorlevel 1 (
    echo [ERR] QEMU variant build failed.
    pause
    exit /b 1
)

echo [OK] P-03_rgb_ws2812 QEMU/SIM variant compiled.
echo.
echo [NOTE] QEMU boot + text assertion is run in the private CI pipeline.
echo        Public copy: build-only. Real hardware: run.bat COMx.
pause
endlocal
