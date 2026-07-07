@echo off
REM ==========================================
REM seven_merge - Windows Build Script (x64)
REM Run: double-click or execute from cmd
REM Prereq: Visual Studio 2022 + CMake
REM ==========================================

setlocal enabledelayedexpansion

REM Resolve project root (parent of scripts/)
set "PROJECT_DIR=%~dp0.."
cd /d "%PROJECT_DIR%"

echo.
echo ===== Checking Build Environment =====

where cmake >nul 2>&1
if %ERRORLEVEL% neq 0 (
    echo [ERROR] CMake not found. Install CMake and add to PATH.
    pause
    exit /b 1
)
echo [OK] CMake found

echo.
echo ===== Building (Release x64) =====
echo Project: %CD%

if exist build rmdir /s /q build
mkdir build
cd build

cmake .. -G "Visual Studio 17 2022" -A x64 -DCMAKE_BUILD_TYPE=Release
if %ERRORLEVEL% neq 0 (
    echo [ERROR] CMake configure failed!
    pause
    exit /b 1
)

findstr /C:"CMAKE_GENERATOR_PLATFORM:INTERNAL=win32" CMakeCache.txt >nul 2>&1
if %ERRORLEVEL% equ 0 (
    echo [ERROR] Configured as x86, not x64! Delete build dir and retry.
    pause
    exit /b 1
)
echo [OK] Architecture confirmed: x64

cmake --build . --config Release
if %ERRORLEVEL% neq 0 (
    echo [ERROR] Build failed!
    pause
    exit /b 1
)

echo.
echo ===== Build Succeeded =====
echo [OK] Output: build\bin\Release\app.exe

cd ..
pause
