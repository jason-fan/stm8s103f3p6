@echo off
REM ============================================================================
REM  STM8S103F3P6 LED Blink - Windows Build Script (SDCC)
REM ============================================================================
REM  Prerequisites:
REM    1. Install SDCC: https://sdcc.sourceforge.net/
REM    2. Ensure sdcc.exe is in your PATH
REM ============================================================================

setlocal enabledelayedexpansion

set PROJECT=led_blink
set SRC1=%PROJECT%_main.c
set SRC2=led.c
set SRC3=key.c
set SRC4=uart.c
set SRC5=eeprom.c
set SRC6=adc.c
set SRC7=beep.c
set REL1=led.rel
set REL2=%PROJECT%_main.rel
set REL3=key.rel
set REL4=uart.rel
set REL5=eeprom.rel
set REL6=adc.rel
set REL7=beep.rel
set OUTPUT=%PROJECT%.ihx
set HEX=%PROJECT%.hex

echo.
echo ========================================
echo   STM8S103 LED Blink - Build Script
echo ========================================
echo.

REM --- Check if SDCC is installed ---
where sdcc >nul 2>&1
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] SDCC not found in PATH.
    echo         Please install SDCC from https://sdcc.sourceforge.net/
    pause
    exit /b 1
)

echo [INFO] Compiler: SDCC
sdcc --version 2>&1 | findstr /C:"SDCC"
echo.

REM --- Check if source files exist ---
for %%F in ("%SRC1%" "%SRC2%" "%SRC3%" "%SRC4%" "%SRC5%" "%SRC6%" "%SRC7%") do (
    if not exist %%~F (
        echo [ERROR] Source file '%%~nxF' not found.
        pause
        exit /b 1
    )
)

echo [INFO] Sources: %SRC1%  %SRC2%  %SRC3%  %SRC4%  %SRC5%  %SRC6%  %SRC7%
echo.

REM ====== Compile ======
echo [STEP] Compiling %SRC2%...
sdcc -mstm8 -c %SRC2%
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Compilation of %SRC2% failed!
    pause
    exit /b 1
)

echo [STEP] Compiling %SRC3%...
sdcc -mstm8 -c %SRC3%
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Compilation of %SRC3% failed!
    pause
    exit /b 1
)

echo [STEP] Compiling %SRC4%...
sdcc -mstm8 -c %SRC4%
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Compilation of %SRC4% failed!
    pause
    exit /b 1
)

echo [STEP] Compiling %SRC5%...
sdcc -mstm8 -c %SRC5%
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Compilation of %SRC5% failed!
    pause
    exit /b 1
)

echo [STEP] Compiling %SRC6%...
sdcc -mstm8 -c %SRC6%
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Compilation of %SRC6% failed!
    pause
    exit /b 1
)

echo [STEP] Compiling %SRC7%...
sdcc -mstm8 -c %SRC7%
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Compilation of %SRC7% failed!
    pause
    exit /b 1
)

echo [STEP] Compiling %SRC1%...
sdcc -mstm8 -c %SRC1%
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Compilation of %SRC1% failed!
    pause
    exit /b 1
)

echo [STEP] Linking...
sdcc -mstm8 %REL2% %REL1% %REL3% %REL4% %REL5% %REL6% %REL7% -o "%OUTPUT%"

if %ERRORLEVEL% NEQ 0 (
    echo.
    echo [ERROR] Compilation failed! Check errors above.
    pause
    exit /b 1
)

echo [ OK ] Compilation successful.

REM ====== Generate Intel HEX ======
if exist "%OUTPUT%" (
    echo [STEP] Generating Intel HEX...
    packihx "%OUTPUT%" > "%HEX%"
    if exist "%HEX%" (
        echo [ OK ] HEX file generated: %HEX%
    ) else (
        echo [WARN] packihx may have failed. HEX not generated.
    )
)

REM ====== Summary ======
echo.
echo ========================================
echo   Build Summary
echo ========================================
if exist "%OUTPUT%" echo   %OUTPUT%  -  IHX output
if exist "%HEX%"    echo   %HEX%     -  Intel HEX file
if exist "%PROJECT%.lk"   echo   %PROJECT%.lk    -  linker script
if exist "%PROJECT%.map"  echo   %PROJECT%.map   -  memory map
if exist "%PROJECT%.lst"  echo   %PROJECT%.lst   -  assembly listing
echo ========================================
echo.

REM ====== Flash options ======
echo Flash commands:
echo   ST-LINK/V2:  stm8flash -c stlinkv2 -p stm8s103f3 -w %OUTPUT%
echo   UART BOOT:   stm8gal -p COMx -w %HEX% -R 3
echo.

REM ====== Clean up intermediate files ======
echo [STEP] Cleaning intermediate files...
for %%x in (asm lst rst rel sym map lk mem) do (
    if exist "%PROJECT%.%%x"  del "%PROJECT%.%%x"
    if exist "%PROJECT%_main.%%x" del "%PROJECT%_main.%%x"
    if exist "led.%%x"        del "led.%%x"
    if exist "key.%%x"        del "key.%%x"
    if exist "uart.%%x"       del "uart.%%x"
    if exist "eeprom.%%x"     del "eeprom.%%x"
    if exist "adc.%%x"        del "adc.%%x"
    if exist "beep.%%x"       del "beep.%%x"
)
echo [ OK ] Intermediate files removed.

echo.
pause
endlocal
