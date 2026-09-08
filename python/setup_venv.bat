@echo off
rem (c) Jan Zwiener (jan@zwiener.org)
REM One-time setup of a venv for python\replay.py: installs PyYAML,
REM pymavlink and the INSLIB package itself as an editable install.
REM Needs mingw32-make on PATH to build libINSLIB if it isn't built yet.
REM
REM Usage:
REM   python\setup_venv.bat              creates python\.venv
REM   python\setup_venv.bat C:\path\env  custom venv location
REM
REM Then:
REM   python\.venv\Scripts\activate.bat
REM   python python\replay.py datasets\fog --realtime
setlocal
cd /d "%~dp0\.."

set "VENV=%~1"
if "%VENV%"=="" set "VENV=python\.venv"

if not exist python\INSLIB\libINSLIB.dll (
    echo building libINSLIB ^(mingw32-make pylib^)...
    mingw32-make pylib || goto :error
)

echo creating venv at %VENV% ...
python -m venv "%VENV%" || goto :error

call "%VENV%\Scripts\activate.bat" || goto :error

python -m pip install --upgrade pip >nul
pip install -r python\requirements.txt || goto :error
pip install -e python\ || goto :error

echo.
echo done. activate with:
echo   %VENV%\Scripts\activate.bat
echo then e.g.:
echo   python python\replay.py datasets\fog --realtime
goto :eof

:error
echo setup_venv.bat failed (see above).
exit /b 1
