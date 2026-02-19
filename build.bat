@echo off
echo ============================================
echo  WiggleMe AI - Build Script
echo ============================================

REM Try to find Visual Studio Build Tools
if exist "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" (
    call "C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
) else if exist "C:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" (
    call "C:\Program Files\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
) else if exist "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" (
    call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat" >nul 2>&1
) else (
    echo ERROR: Visual Studio Build Tools not found!
    echo Install from: https://visualstudio.microsoft.com/visual-cpp-build-tools/
    exit /b 1
)

echo.
echo Compiling WiggleMe AI...
cl /EHsc /W4 /O2 /std:c++17 /DUNICODE /D_UNICODE ^
    /I"include" ^
    src\main.cpp src\ai_movement.cpp ^
    user32.lib gdi32.lib comctl32.lib shell32.lib shlwapi.lib ^
    /Fe:WiggleMeAI.exe ^
    /link /SUBSYSTEM:WINDOWS

if %errorlevel% neq 0 (
    echo.
    echo BUILD FAILED!
    exit /b 1
)

echo.
echo ============================================
echo  BUILD SUCCESS: WiggleMeAI.exe
echo ============================================
echo.

REM Clean up intermediate files
del /q *.obj >nul 2>&1

echo Done!
