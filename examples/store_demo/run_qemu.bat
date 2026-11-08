@echo off
rem =====================================================================
rem  store_demo -- SIM variant build (compile check, no board required)
rem
rem  Builds the synthetic-data variant (-DEXP_SIM=1 -DEXP_QEMU_UART=1). Booting it
rem  in QEMU and asserting on the serial output belongs to the project's
rem  private CI; this public copy only verifies that the variant compiles.
rem
rem  For a real board use run.bat COMx.
rem =====================================================================
setlocal
cd /d "%~dp0"
set MSYSTEM=
if not defined IDF_PATH set IDF_PATH=C:\esp\v6.1\esp-idf
if not defined IDF_PYTHON_ENV_PATH set IDF_PYTHON_ENV_PATH=C:\Espressif\python_env\idf6.1_py3.11_env
if not exist "%IDF_PATH%\export.bat" (
    echo [ERR] ESP-IDF SDK not found: %IDF_PATH%\export.bat
    pause
    exit /b 1
)
call "%IDF_PATH%\export.bat" >nul
if errorlevel 1 (
    echo [ERR] Failed to activate ESP-IDF environment.
    pause
    exit /b 1
)
idf.py -B build-sim -DEXP_SIM=1 -DEXP_QEMU_UART=1 build
if errorlevel 1 (
    echo [ERR] SIM variant build failed.
    pause
    exit /b 1
)
echo [OK] store_demo SIM variant compiled.
pause
endlocal
