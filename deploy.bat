@echo off
title FTC FAST DEPLOY - Team 9000
color 0a
cls

echo.
echo  =============================================
echo  ===       FTC SUPER-FAST DEPLOY v2        ===
echo  ===            Team 9000                  ===
echo  =============================================
echo.

:: ========= AUTO-FIND ADB (works everywhere) =========
set "ADB=%LOCALAPPDATA%\Android\Sdk\platform-tools\adb.exe"
if not exist "%ADB%" (
    echo [ERROR] adb.exe not found!
    echo.
    echo Please install Android Studio or Platform-Tools from:
    echo https://developer.android.com/tools/releases/platform-tools
    echo.
    pause
    exit /b 1
)

:: ========= CONNECT TO CONTROL HUB =========
echo Connecting to Control Hub...
"%ADB%" connect 192.168.43.1:5555 >nul 2>&1
timeout /t 2 >nul

"%ADB%" devices | findstr /R /C:"192.168.43.1.*device" >nul