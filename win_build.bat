::
:: win_build.bat
::
:: Batch file to locally build under Windows using Conda
::
:: See README.md for details
::
setlocal EnableDelayedExpansion
@echo on

:: Set number of CPUs to use for build
set CPU_COUNT=3

:: Build RtAudio
:: cd thirdparty/rtaudio

:: Make a build folder and change to it
:: mkdir build
:: cd build

:: configure
:: cmake -G "Ninja" ^
::     -DCMAKE_BUILD_TYPE:STRING=Release ^
::     -DCMAKE_INSTALL_PREFIX:PATH="%LIBRARY_PREFIX%" ^
::     -DCMAKE_PREFIX_PATH:PATH="%LIBRARY_PREFIX%" ^
    ..
:: if errorlevel 1 exit /B 1

:: build
:: cmake --build . --config Release -- -j%CPU_COUNT%
:: if errorlevel 1 exit /B 1

::go back to m17-tools
:: cd ../../../

:: Make a build folder and change to it
mkdir build
cd build

:: configure
cmake -G "Ninja" ^
    -DCMAKE_BUILD_TYPE:STRING=Release ^
    -DCMAKE_INSTALL_PREFIX:PATH="%LIBRARY_PREFIX%" ^
    -DCMAKE_PREFIX_PATH:PATH="%LIBRARY_PREFIX%" ^
    ..
if errorlevel 1 exit /B 1

:: build
cmake --build . --config Release -- -j%CPU_COUNT%
if errorlevel 1 exit /B 1

:: install
cmake --build . --config Release --target install
if errorlevel 1 exit /B 1

:: test
ctest --build-config Release --output-on-failure --timeout 120 -j%CPU_COUNT%
if errorlevel 1 exit /B 1
