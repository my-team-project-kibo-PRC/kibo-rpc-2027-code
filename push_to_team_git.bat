@echo off
title Push kibo-PRC to Team Repository
echo ============================================================
echo   Syncing kibo-PRC to Team GitHub Repository
echo   Remote URL: https://github.com/my-team-project-kibo-PRC/kibo-rpc-2027-code
echo   Author: korb sameth
echo ============================================================
echo.
cd /d "%~dp0"

echo Choose an option:
echo [1] Push to branch 'sameth-dev' (Recommended for teamwork)
echo [2] Push directly to branch 'main' (Force update)
echo [3] Push to both personal GitHub and team GitHub
echo.
set /p choice="Enter option (1, 2, or 3): "

if "%choice%"=="1" (
    echo.
    echo Pushing local code to team remote on branch 'sameth-dev'...
    git push team main:sameth-dev
) else if "%choice%"=="2" (
    echo.
    echo Pushing local code to team remote on branch 'main'...
    git push team main:main --force
) else if "%choice%"=="3" (
    echo.
    echo 1/2 Pushing to personal repository (origin)...
    git push origin main
    echo.
    echo 2/2 Pushing to team repository (team:sameth-dev)...
    git push team main:sameth-dev
) else (
    echo.
    echo Defaulting to Option 1: Pushing to branch 'sameth-dev'...
    git push team main:sameth-dev
)

echo.
if %ERRORLEVEL% equ 0 (
    echo [SUCCESS] Push completed successfully!
) else (
    echo [ERROR] Push failed. Please check your GitHub permissions for the team repository.
)
echo.
pause
