@echo off
setlocal

REM ELEC 442 personal-computer setup
REM This script sets QARM_LAB_ENV to the folder containing this file.
REM Keep this file in the root of the lab-machine-env repository.

for %%I in ("%~dp0.") do set "LAB_ENV=%%~fI"

REM Verify the health of the lab-machine-env repository

if not exist "%LAB_ENV%\QArm-control" (
    echo ERROR: QArm-control was not found in:
    echo   %LAB_ENV%
    echo.
    echo Keep this script in the root of the lab-machine-env repository.
    exit /b 1
)

if not exist "%LAB_ENV%\Lab-1" (
    echo ERROR: Lab-1 was not found in:
    echo   %LAB_ENV%
    echo.
    echo Keep this script in the root of the lab-machine-env repository.
    exit /b 1
)

echo Setting QARM_LAB_ENV to:
echo   %LAB_ENV%
echo.

setx QARM_LAB_ENV "%LAB_ENV%" >nul

if errorlevel 1 (
    echo ERROR: QARM_LAB_ENV could not be set.
    exit /b 1
)

echo QARM_LAB_ENV was configured successfully.
echo.
echo Close and reopen VS Code and any terminals before continuing.
echo A computer restart is not required.

endlocal
