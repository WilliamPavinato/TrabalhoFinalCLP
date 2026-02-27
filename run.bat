@echo off
setlocal

set PYTHON_CMD=python
set CARGO_CMD=cargo
set LIB_WIN=target\release\missile_sim.dll

:menu
cls
echo ============================================================
echo   ICBM Sim - Windows Automation (Equivalent to Makefile)
echo ============================================================
echo  1. Build (Compilar motor em Rust)
echo  2. GUI   (Rodar interface Pygame)
echo  3. Study (Rodar simulacao Headless e gerar CSV)
echo  4. Check (Verificar dependencias)
echo  5. Clean (Limpar arquivos)
echo  6. Sair
echo ============================================================
set /p opt="Escolha uma opcao: "

if %opt%==1 goto build
if %opt%==2 goto gui
if %opt%==3 goto study
if %opt%==4 goto check
if %opt%==5 goto clean
if %opt%==6 goto end

:build
echo ==^> Building Rust shared library...
%CARGO_CMD% build --release
if exist %LIB_WIN% (
    copy %LIB_WIN% .
    echo Copied missile_sim.dll to project root.
)
pause
goto menu

:gui
if not exist missile_sim.dll (
    echo [AVISO] DLL nao encontrada. Compilando primeiro...
    call :build
)
echo ==^> Launching GUI...
%PYTHON_CMD% MissileSimulation.py
pause
goto menu

:study
if not exist missile_sim.dll (
    echo [AVISO] DLL nao encontrada. Compilando primeiro...
    call :build
)
echo ==^> Running case-of-study (headless trajectory, prints CSV)...
%PYTHON_CMD% run_study.py
pause
goto menu

:check
echo Checking dependencies...
%CARGO_CMD% --version || echo ERROR: Cargo/Rust not found.
%PYTHON_CMD% --version || echo ERROR: Python not found.
%PYTHON_CMD% -c "import pygame; print('pygame ok')" || echo ERROR: Pygame not found.
pause
goto menu

:clean
echo Cleaning...
%CARGO_CMD% clean
del /f missile_sim.dll trajectory_output.csv 2>nul
pause
goto menu

:end
exit