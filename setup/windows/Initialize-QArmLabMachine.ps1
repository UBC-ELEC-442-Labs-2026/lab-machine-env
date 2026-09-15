#Requires -Version 5.1
#Requires -RunAsAdministrator

[CmdletBinding()]
param(
    [string]$QuanserResources =
        'C:\Quanser',

    [string]$LabMachineEnvironment =
        'C:\QArmLab\lab-machine-env',

    [string]$BasePython =
        'C:\Program Files\Python313\python.exe',

    [string]$VenvDirectory =
        'C:\ProgramData\Qarm\Python\venv'
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

function Assert-Path {
    param(
        [Parameter(Mandatory)]
        [string]$Path,

        [Parameter(Mandatory)]
        [string]$Description
    )

    if (-not (Test-Path -LiteralPath $Path)) {
        throw "$Description was not found at: $Path"
    }
}

function Invoke-External {
    param(
        [Parameter(Mandatory)]
        [string]$FilePath,

        [Parameter(Mandatory)]
        [string[]]$ArgumentList,

        [Parameter(Mandatory)]
        [string]$Description
    )

    Write-Host "`n==> $Description" -ForegroundColor Cyan

    & $FilePath @ArgumentList

    if ($LASTEXITCODE -ne 0) {
        throw "$Description failed with exit code $LASTEXITCODE."
    }
}

function Set-MachineVariable {
    param(
        [Parameter(Mandatory)]
        [string]$Name,

        [Parameter(Mandatory)]
        [string]$Value
    )

    [Environment]::SetEnvironmentVariable(
        $Name,
        $Value,
        [System.EnvironmentVariableTarget]::Machine
    )

    # Also expose it to commands launched by this script.
    Set-Item -Path "Env:$Name" -Value $Value

    Write-Host "$Name = $Value"
}

Write-Host 'QArm shared Windows environment setup' -ForegroundColor Green

$QuanserRequirements = Join-Path `
    $QuanserResources '1_setup\requirements.txt'

$AcademicPython = Join-Path `
    $QuanserResources '0_libraries\python'

$RtModelsDirectory = Join-Path `
    $QuanserResources '0_libraries\resources\rt_models'

$QArmSupportPython = Join-Path `
    $LabMachineEnvironment 'QArm-control'

Assert-Path $QuanserResources 'Quanser Academic Resources'
Assert-Path $QuanserRequirements 'Quanser requirements.txt'
Assert-Path $AcademicPython 'Quanser Academic Resources Python library'
Assert-Path $RtModelsDirectory 'Quanser real-time models'
Assert-Path $LabMachineEnvironment 'lab-machine-env repository'
Assert-Path $QArmSupportPython 'QArm-control support directory'
Assert-Path $BasePython 'Machine-wide Python 3.13 interpreter'

$QsdkDirectory = [Environment]::GetEnvironmentVariable(
    'QSDK_DIR',
    [System.EnvironmentVariableTarget]::Machine
)

if ([string]::IsNullOrWhiteSpace($QsdkDirectory)) {
    $QsdkDirectory = $env:QSDK_DIR
}

if ([string]::IsNullOrWhiteSpace($QsdkDirectory)) {
    $QsdkDirectory = 'C:\Program Files\Quanser\QUARC'
}

$QsdkPython = Join-Path $QsdkDirectory 'python'
Assert-Path $QsdkPython 'Quanser API wheel directory'

$WheelCandidates = @(
    Get-ChildItem `
        -LiteralPath $QsdkPython `
        -Filter 'quanser_api*.whl' `
        -File
)

if ($WheelCandidates.Count -eq 0) {
    throw "No quanser_api wheel was found in $QsdkPython"
}

if ($WheelCandidates.Count -gt 1) {
    Write-Warning 'Multiple Quanser API wheels were found; using the newest file.'
}

$QuanserWheel = $WheelCandidates |
    Sort-Object LastWriteTime -Descending |
    Select-Object -First 1

Write-Host "Quanser API wheel: $($QuanserWheel.FullName)"

$VersionOutput = & $BasePython --version 2>&1

if ($LASTEXITCODE -ne 0) {
    throw 'Could not determine the base Python version.'
}

$PythonVersion = ($VersionOutput |
    Select-Object -First 1).ToString().Trim()

if ($PythonVersion -notmatch '^Python 3\.13(?:\.|$)') {
    throw "Expected Python 3.13, but $BasePython reports $PythonVersion."
}

Write-Host "Base Python: $PythonVersion"

$VenvParent = Split-Path -Parent $VenvDirectory
New-Item -ItemType Directory -Force $VenvParent | Out-Null

$VenvPython = Join-Path $VenvDirectory 'Scripts\python.exe'

if (-not (Test-Path -LiteralPath $VenvPython)) {
    Invoke-External `
        -FilePath $BasePython `
        -ArgumentList @('-m', 'venv', $VenvDirectory) `
        -Description 'Creating the shared Python virtual environment'
}
else {
    $VenvVersionOutput = & $VenvPython --version 2>&1

    if ($LASTEXITCODE -ne 0) {
        throw "The existing environment at $VenvDirectory is incomplete."
    }

    $VenvVersion = ($VenvVersionOutput |
        Select-Object -First 1).ToString().Trim()

    if ($VenvVersion -notmatch '^Python 3\.13(?:\.|$)') {
        throw "The existing environment reports $VenvVersion, not Python 3.13."
    }

    Write-Host "Existing environment found: $VenvVersion"
}

Invoke-External `
    -FilePath $VenvPython `
    -ArgumentList @('-m', 'pip', 'install', '--upgrade', 'pip') `
    -Description 'Updating pip'

Push-Location -LiteralPath $QsdkPython

try {
    Invoke-External `
        -FilePath $VenvPython `
        -ArgumentList @(
            '-m',
            'pip',
            'install',
            '--upgrade',
            '--find-links=.',
            $QuanserWheel.Name
        ) `
        -Description 'Installing the Quanser Python API'
}
finally {
    Pop-Location
}

# Lab 3 requires cv2.aruco. Both OpenCV distributions install the
# same cv2 package, so omit opencv-python from Quanser's requirements
# and install opencv-contrib-python instead.
$TemporaryRequirements = Join-Path `
    ([IO.Path]::GetTempPath()) `
    ("qarm-requirements-{0}.txt" -f [Guid]::NewGuid().ToString('N'))

try {
    Get-Content -LiteralPath $QuanserRequirements |
        Where-Object {
            $_ -notmatch '^\s*opencv-python(?:\s*[<>=!~].*)?\s*$'
        } |
        Set-Content `
            -LiteralPath $TemporaryRequirements `
            -Encoding ASCII

    Invoke-External `
        -FilePath $VenvPython `
        -ArgumentList @(
            '-m',
            'pip',
            'install',
            '-r',
            $TemporaryRequirements
        ) `
        -Description 'Installing Quanser Academic Resources dependencies'

    Invoke-External `
        -FilePath $VenvPython `
        -ArgumentList @(
            '-m',
            'pip',
            'install',
            '--upgrade',
            'numpy<2.4',
            'opencv-contrib-python',
            'pyrealsense2'
        ) `
        -Description 'Installing lab-specific vision dependencies'
}
finally {
    Remove-Item `
        -LiteralPath $TemporaryRequirements `
        -Force `
        -ErrorAction SilentlyContinue
}

$SitePackagesOutput = & $VenvPython -c `
    'import site; print(site.getsitepackages()[0])'

if ($LASTEXITCODE -ne 0) {
    throw 'Could not locate the shared environment site-packages directory.'
}

$SitePackages = ($SitePackagesOutput |
    Select-Object -First 1).ToString().Trim()

$PathFile = Join-Path $SitePackages 'qarm_lab_paths.pth'

@(
    $AcademicPython
    $QArmSupportPython
) | Set-Content -LiteralPath $PathFile -Encoding ASCII

Write-Host "`nCreated Python path file: $PathFile"

Set-MachineVariable -Name 'QAL_DIR' -Value $QuanserResources
Set-MachineVariable -Name 'RTMODELS_DIR' -Value $RtModelsDirectory
Set-MachineVariable -Name 'QARM_LAB_ENV' -Value $LabMachineEnvironment
Set-MachineVariable -Name 'QARM_PYTHON' -Value $VenvPython

$VerificationCode = @'
import cv2
import numpy
import pyrealsense2

from quanser.hardware import HIL
from pal.products.qarm import QArm
from hal.products.qarm import QArmUtilities
from QArm_functions import QArm_Lab_interface

assert hasattr(cv2, "aruco"), "cv2.aruco is unavailable"

print("NumPy:", numpy.__version__)
print("OpenCV:", cv2.__version__)
print("RealSense import: OK")
print("Quanser API import: OK")
print("PAL/HAL imports: OK")
print("Course QArm wrapper import: OK")
'@

$TemporaryVerification = Join-Path `
    ([IO.Path]::GetTempPath()) `
    ("verify-qarm-{0}.py" -f [Guid]::NewGuid().ToString('N'))

try {
    Set-Content `
        -LiteralPath $TemporaryVerification `
        -Value $VerificationCode `
        -Encoding ASCII

    Invoke-External `
        -FilePath $VenvPython `
        -ArgumentList @($TemporaryVerification) `
        -Description 'Verifying Python imports without opening hardware'
}
finally {
    Remove-Item `
        -LiteralPath $TemporaryVerification `
        -Force `
        -ErrorAction SilentlyContinue
}

Invoke-External `
    -FilePath $VenvPython `
    -ArgumentList @('-m', 'pip', 'check') `
    -Description 'Checking Python dependency consistency'

$ManifestPath = Join-Path $VenvParent 'installed-packages.txt'
$FreezeOutput = & $VenvPython -m pip freeze

if ($LASTEXITCODE -ne 0) {
    throw 'Could not record the installed Python packages.'
}

$FreezeOutput |
    Set-Content -LiteralPath $ManifestPath -Encoding UTF8

Write-Host "`nSetup completed successfully." -ForegroundColor Green
Write-Host "Shared interpreter: $VenvPython"
Write-Host "Package manifest:   $ManifestPath"