@echo off
rem ===========================================================================
rem  build_pylib_msvc.bat -- build python\INSLIB\libINSLIB.dll with MSVC.
rem
rem  Windows fallback for `make pylib` when gcc is not installed. Mirrors the
rem  Makefile pylib target's source list and produces the same DLL the INSLIB
rem  python package loads. Run from anywhere (uses its own directory):
rem
rem      build_pylib_msvc.bat
rem
rem  Needs Visual Studio with the "Desktop development with C++" (x64)
rem  workload. MSVC specifics vs. gcc: _USE_MATH_DEFINES for M_PI, and an
rem  explicit exports .def (MSVC does not auto-export symbols like gcc).
rem ===========================================================================
setlocal
pushd "%~dp0"

rem --- locate and enter the MSVC x64 build environment (vcvars64) -----------
set "VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%VSWHERE%" (
    echo ERROR: vswhere.exe not found -- is Visual Studio installed?
    goto :fail
)
rem  -prerelease so Preview/Insiders installs (e.g. VS 18 Insiders) are found
rem  too -- vswhere hides those by default and would otherwise report nothing.
set "VSPATH="
for /f "usebackq tokens=*" %%i in (`"%VSWHERE%" -latest -prerelease -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VSPATH=%%i"
if not defined VSPATH (
    echo ERROR: no Visual Studio with the C++ x64 toolchain was found.
    goto :fail
)
call "%VSPATH%\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 ( echo ERROR: vcvars64.bat failed & goto :fail )

rem --- compile (keep this source list in sync with the Makefile) ------------
set "INCS=/I python\csrc /I src /I KFCore\c /I KFCore\c\navigation_tools /I KFCore\tests"
set "SRCS=src\ins.c src\geodetic_toolbox.c src\magnetic_model.c src\ahrs.c src\baro_alt.c src\nav_suite.c src\log.c KFCore\c\linalg.c KFCore\c\kalman_udu.c KFCore\c\miniblas.c python\csrc\ins_capi.c"

if not exist build_cl mkdir build_cl
echo [1/3] compiling...
cl /nologo /std:c11 /O2 /c /D_USE_MATH_DEFINES /wd4068 %INCS% %SRCS% /Fo:build_cl\
if errorlevel 1 ( echo ERROR: compile failed & goto :fail )

rem --- generate the exports .def (the ins_core_*/ins_suite_* capi + magnetic_*
rem     and ins_gravity_ned helpers that the python package binds; gcc would
rem     export these automatically) -----------------------------------------
echo [2/3] generating exports...
set "DEF=python\INSLIB\libINSLIB.def"
echo EXPORTS> "%DEF%"
for /f "tokens=2 delims=|" %%s in ('dumpbin /SYMBOLS build_cl\ins_capi.obj ^| findstr "External" ^| findstr "() "') do @echo  %%s>> "%DEF%"
for /f "tokens=2 delims=|" %%s in ('dumpbin /SYMBOLS build_cl\magnetic_model.obj ^| findstr "External" ^| findstr "() " ^| findstr "magnetic"') do @echo  %%s>> "%DEF%"
for /f "tokens=2 delims=|" %%s in ('dumpbin /SYMBOLS build_cl\geodetic_toolbox.obj ^| findstr "External" ^| findstr "() " ^| findstr "ins_gravity_ned"') do @echo  %%s>> "%DEF%"

rem --- link the shared library ----------------------------------------------
echo [3/3] linking...
link /nologo /DLL /DEF:"%DEF%" /OUT:python\INSLIB\libINSLIB.dll build_cl\*.obj
if errorlevel 1 ( echo ERROR: link failed & goto :fail )

rem --- clean intermediates --------------------------------------------------
del /q build_cl\*.obj >nul 2>&1
rmdir build_cl >nul 2>&1
del /q "%DEF%" python\INSLIB\libINSLIB.exp python\INSLIB\libINSLIB.lib >nul 2>&1

echo.
echo OK: python\INSLIB\libINSLIB.dll built.
echo Verify with:  python -c "import sys; sys.path.insert(0,'python'); import INSLIB; print('import OK')"
popd
endlocal
exit /b 0

:fail
popd
endlocal
exit /b 1
