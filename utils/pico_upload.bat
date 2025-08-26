@echo off
echo ===== Pi Pico Renderer Quick Upload =====

REM Get COM port from command line argument, default to COM4 if not provided
set COM_PORT=%2
if "%COM_PORT%"=="" set COM_PORT=COM4

echo.
echo Attempting to reset Pi Pico via %COM_PORT%...
powershell -Command "& {try { $port = New-Object System.IO.Ports.SerialPort('%COM_PORT%', 1200); $port.Open(); Start-Sleep -Milliseconds 100; $port.Close(); Write-Host 'Reset signal sent successfully' } catch { Write-Host 'Port %COM_PORT% not available - Pico may now be in BOOTSEL mode' }}"

echo Waiting for Pi Pico to enter BOOTSEL mode...
ping 127.0.0.1 -n 4 >nul

echo Looking for Pi Pico drive...

for %%d in (D E F G H I J K L M N O P Q R S T U V W X Y Z) do (
    if exist %%d:\INFO_UF2.TXT (
        echo Found Pi Pico at %%d:\
        copy ".pio\build\Renderer\firmware.uf2" "%%d:\" >nul
        if errorlevel 1 (
            echo ERROR: Copy failed!
            exit /b 1
        ) else (
            echo SUCCESS: Firmware uploaded!
            goto :end
        )
    )
)

echo ERROR: Pi Pico drive not found! Make sure it's in BOOTSEL mode and nothing blocks the USB connection for example a serial monitor.
exit /b 1

:end
echo Upload complete!
exit /b 0
