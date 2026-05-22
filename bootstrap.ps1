# Toolchain setup script for Windows environments (Experimental / Untested)
# Automated local self-contained workspace deployment for Puya PY32F0xx

$ErrorActionPreference = "Stop"

Write-Host "=================================================================" -ForegroundColor Cyan
Write-Host " Deploying Local Environment for Puya PY32F0xx (Windows) " -ForegroundColor Cyan
Write-Host "=================================================================" -ForegroundColor Cyan

# 1. Local path configuration
$BaseDir = Get-Location
$ToolchainDir = Join-Path $BaseDir "toolchain"
$GccDir = Join-Path $ToolchainDir "gcc-arm"
$PuyaTemplateDir = Join-Path $ToolchainDir "py32f0"
$PyocdDir = Join-Path $ToolchainDir "pyocd"
$ScriptsDir = Join-Path $ToolchainDir "scripts"

# ARM GCC 14.2.rel1 Toolchain configuration for Windows
$GccVersion = "14.2.rel1"
$GccZip = "arm-gnu-toolchain-$GccVersion-mingw-w64-i686-arm-none-eabi.zip"
$GccUrl = "https://developer.arm.com/-/media/Files/downloads/gnu/$GccVersion/binrel/$GccZip"

Write-Host "➡️ Creating directory tree under toolchain\\..." -ForegroundColor Yellow
New-Item -ItemType Directory -Force -Path $GccDir | Out-Null
New-Item -ItemType Directory -Force -Path $PuyaTemplateDir | Out-Null
New-Item -ItemType Directory -Force -Path $PyocdDir | Out-Null
New-Item -ItemType Directory -Force -Path $ScriptsDir | Out-Null

# 2. Local download and extraction of ARM GCC
Write-Host "➡️ Verifying local ARM GCC compiler..." -ForegroundColor Yellow
$ExpectedGccFolder = Join-Path $GccDir "arm-gnu-toolchain-$GccVersion-mingw-w64-i686-arm-none-eabi"
if (-not (Test-Path $ExpectedGccFolder)) {
    Write-Host "   Downloading ARM GNU Toolchain for Windows ($GccVersion)..." -ForegroundColor White
    $ZipDest = Join-Path $GccDir $GccZip
    Invoke-WebRequest -Uri $GccUrl -OutFile $ZipDest -Verbose
    
    Write-Host "   Extracting compiler package..." -ForegroundColor White
    Expand-Archive -Path $ZipDest -DestinationPath $GccDir -Force
    Remove-Item $ZipDest
    Write-Host "   [OK] Local toolchain extraction complete." -ForegroundColor Green
} else {
    Write-Host "   [OK] Compiler already exists inside local workspace." -ForegroundColor Green
}

# 3. Clone Puya LL peripheral template repository
Write-Host "➡️ Verifying PY32F0xx LL support library..." -ForegroundColor Yellow
$TemplateDest = Join-Path $PuyaTemplateDir "py32f0-template"
if (-not (Test-Path $TemplateDest)) {
    Write-Host "   Cloning IOsetting/py32f0-template via Git..." -ForegroundColor White
    Set-Location $PuyaTemplateDir
    git clone https://github.com/IOsetting/py32f0-template.git py32f0-template
    Set-Location $BaseDir
    Write-Host "   [OK] Puya template repository cloned." -ForegroundColor Green
} else {
    Write-Host "   [OK] Library template already present." -ForegroundColor Green
}

# 4. Configure isolated Python Virtual Environment for pyOCD utilities
Write-Host "➡️ Building isolated Python virtual environment for pyOCD..." -ForegroundColor Yellow
$VenvPath = Join-Path $PyocdDir "venv"
if (-not (Test-Path $VenvPath)) {
    Write-Host "   Creating venv at toolchain\\pyocd\\venv..." -ForegroundColor White
    python -m venv $VenvPath
    
    Write-Host "   Installing pyOCD packages inside local venv..." -ForegroundColor White
    $PipExe = Join-Path $VenvPath "Scripts\\pip.exe"
    & $PipExe install -U pip setuptools
    & $PipExe install -U pyocd
    Write-Host "   [OK] pyOCD successfully bound to isolated venv." -ForegroundColor Green
} else {
    Write-Host "   [OK] Virtual environment configuration already matching." -ForegroundColor Green
}

# 5. Automatically deploy unified configuration layout
Write-Host "➡️ Deploying workspace configurations (pyocd.yaml)..." -ForegroundColor Yellow
$YamlContent = @"
# Auto-generated pyocd configuration file for local workspace execution
pack:
  - ./toolchain/py32f0/py32f0-template/Misc/Puya.PY32F0xx_DFP.1.1.7.pack

target_override: py32f002ax5

debug:
  connect_mode: under-reset
"@

$YamlContent | Out-File -FilePath (Join-Path $BaseDir "pyocd.yaml") -Encoding ascii -Force
$YamlContent | Out-File -FilePath (Join-Path $PyocdDir "pyocd.yaml") -Encoding ascii -Force
Write-Host "   [OK] Local configurations generated successfully." -ForegroundColor Green

Write-Host "=================================================================" -ForegroundColor Green
Write-Host " 🎉 Workspace Setup Completed on Windows! " -ForegroundColor Green
Write-Host "=================================================================" -ForegroundColor Green
