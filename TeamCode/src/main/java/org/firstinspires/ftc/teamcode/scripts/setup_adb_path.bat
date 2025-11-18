@echo off
setlocal

REM Hard-code to the path Android Studio is using on your machine
set "ADBPATH=%LOCALAPPDATA%\Android\Sdk\platform-tools"

echo Using ADB path: "%ADBPATH%"

if not exist "%ADBPATH%\adb.exe" (
    echo ❌ adb.exe not found at that location.
    echo Make sure Android Studio is installed and that the "Android SDK Platform-Tools" component is installed.
    pause
    exit /b 1
)

echo ✔ Found adb.exe

REM Just append it to PATH (duplicates are harmless)
echo Adding ADB to your user PATH...
setx PATH "%PATH%;%ADBPATH%" >nul

echo ✔ Done. Close and reopen any Command Prompt, PowerShell, or Android Studio windows.
pause
endlocal
