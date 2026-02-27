@echo off
setlocal

:: --- Configurações ---
set PYTHON_CMD=python
set CARGO_CMD=cargo
set LIB_WIN=target\release\missile_sim.dll

:menu
cls
echo ============================================================
echo   ICBM Sim
echo ============================================================
echo  1. Build (Compilar motor em Rust)
echo  2. GUI   (Rodar interface Pygame)
echo  3. Check (Verificar dependencias)
echo  4. Clean (Limpar arquivos)
echo  5. Sair
echo ============================================================
set /p opt="Escolha uma opcao: "

if %opt%==1 goto build
if %opt%==2 goto gui
if %opt%==3 goto check
if %opt%==4 goto clean
if %opt%==5 goto end

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