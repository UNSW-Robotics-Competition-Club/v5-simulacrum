@echo off
rem V5 brain simulator: VEXos in QEMU plus a simulated robot, from Windows.
rem
rem   V5Sim [preset] [program.bin] [robot options...]
rem
rem   preset       a robot from sim\robots\*.args: drivetrain (default),
rem                competition, pushback-practice, ui-test (on a field
rem                controller, disabled: for testing UI); or a path to any .args file
rem                (e.g. programs\my-bot\robot.args)
rem   program.bin  optional: installed in slot 1 and started once VEXos is up
rem                (write - to give robot options without a program)
rem   robot options  added to the preset, e.g. --match 30 for a field-controlled
rem                match (presets use a radio), --radio 11:partner, or
rem                --physics mujoco (SIMULATOR.md section 4 lists them)
rem
rem   e.g.  V5Sim competition my_auton.bin --match 20
rem   (cmd splits arguments at "=", so quote those: "--start=-36:-60:0")
rem
rem The brain's screen opens in a window (click = touch). The field view is at
rem http://localhost:8765/. With usbip-win2 installed (winget install
rem vadimgrn.usbip-win2) the brain appears on this PC's USB: two COM ports that
rem VEXcode and PROS find as a plugged-in V5 brain. Without it its USB ports are
rem /tmp/ttyV5sys and /tmp/ttyV5user inside WSL (see SIMULATOR.md).
rem If another simulator is already running, this one becomes instance N:
rem field view on port 8765+N, USB ports in /tmp/v5simN/. The "[v5brain]
rem instance" line it prints says where. Set V5SIM_ID=N to pick one.
rem Set V5SIM_HEADLESS=1 to run without the window.
setlocal
rem script dir, captured before any `shift` (shift also shifts %0, so %~dp0
rem would be wrong later in :vstart).
set "HERE=%~dp0"
set "SELF=%~f0"
rem Management verbs:
rem   V5Sim start [preset] [opts]   run an instance in the background, print its id
rem   V5Sim log [id]                follow that instance's console live (Ctrl-C quits)
rem   V5Sim list                    show running instances and their ports
rem   V5Sim status [id]             detail for one instance (default 0)
rem   V5Sim clear [id]              empty all 8 program slots (keep settings)
rem   V5Sim reset [id]              factory reset the brain (wipe its flash)
rem   V5Sim stop <id|all>           graceful shutdown (clean USB detach)
rem   V5Sim attach <id>             (re)connect the brain to Windows USB
rem   V5Sim detach <id|all>         disconnect from Windows USB, leave it up
rem Without a verb, the first word is a preset and the sim runs in this window.
set "VERB=%~1"
if /i "%VERB%"=="start"  goto :vstart
if /i "%VERB%"=="list"   goto :vfleet
if /i "%VERB%"=="ls"     goto :vfleet
if /i "%VERB%"=="status" goto :vfleet
if /i "%VERB%"=="log"    goto :vfleet
if /i "%VERB%"=="logs"   goto :vfleet
if /i "%VERB%"=="tail"   goto :vfleet
if /i "%VERB%"=="clear"  goto :vfleet
if /i "%VERB%"=="reset"  goto :vfleet
if /i "%VERB%"=="stop"   goto :vfleet
if /i "%VERB%"=="attach" goto :vfleet
if /i "%VERB%"=="detach" goto :vfleet
goto :direct

:vfleet
rem delegate to the WSL-side manager, passing the verb and its arguments
for /f "usebackq delims=" %%p in (`wsl -d Ubuntu-24.04 -- wslpath -a "%~dp0scripts\v5fleet.sh"`) do set "FLEET=%%p"
wsl -d Ubuntu-24.04 -- bash "%FLEET%" %*
exit /b %ERRORLEVEL%

:vstart
rem open the sim in its own window; this command returns immediately.
set "REST="
shift
:vstart_more
if "%~1"=="" goto :vstart_go
set "REST=%REST% %1"
shift
goto :vstart_more
:vstart_go
rem Run the instance in the background (a hidden window holds it alive until
rem `v5sim stop`), then report which --id auto instance it claimed. A random
rem token lets v5brain.sh record its id (under ~/re/qemu-runs/win-<token>) so we
rem can read it back; the one-line temp launcher sidesteps `start` arg quoting.
set "TOKEN=%RANDOM%%RANDOM%%RANDOM%"
set "SPAWN=%TEMP%\v5sim_spawn_%RANDOM%.cmd"
> "%SPAWN%" echo @set "V5SIM_WINDOW_TOKEN=%TOKEN%"
>> "%SPAWN%" echo @call "%SELF%"%REST%
>> "%SPAWN%" echo @del "%%~f0"
powershell -NoProfile -Command "Start-Process -WindowStyle Hidden -FilePath '%SPAWN%'"
rem wait for the instance to announce its id, then tell the user how to see it
for /f "usebackq delims=" %%p in (`wsl -d Ubuntu-24.04 -- wslpath -a "%HERE%scripts\v5fleet.sh"`) do set "FLEET=%%p"
set "NEWID="
for /f "usebackq delims=" %%i in (`wsl -d Ubuntu-24.04 -- bash "%FLEET%" waitid %TOKEN%`) do set "NEWID=%%i"
if not defined NEWID (
  echo Started the V5 sim, but could not read its instance id.  Try: v5sim list
  exit /b 0
)
set /a VIEW=8765+NEWID
echo Started V5 sim instance %NEWID%.
echo   field view : http://localhost:%VIEW%/
echo   live log   : v5sim log %NEWID%
echo   stop       : v5sim stop %NEWID%
exit /b 0

:direct
set "PRESET=%~1"
if "%PRESET%"=="" set "PRESET=drivetrain"
rem a path to an .args file (e.g. one kept in a program's folder) is used as-is
if /i not "%PRESET:~-5%"==".args" goto :named
if not exist "%~f1" goto :named
for /f "usebackq delims=" %%p in (`wsl -d Ubuntu-24.04 -- wslpath -a "%~f1"`) do set "PFILE=%%p"
goto :preset_ok
:named
if /i "%PRESET:~-5%"==".args" set "PRESET=%PRESET:~0,-5%"
set "PFILE=robots/%PRESET%.args"
if exist "%~dp0sim\robots\%PRESET%.args" goto :preset_ok
(
  echo No robot preset "%PRESET%". Choose one of:
  dir /b "%~dp0sim\robots\*.args"
  exit /b 2
)
:preset_ok
for /f "usebackq delims=" %%p in (`wsl -d Ubuntu-24.04 -- wslpath -a "%~dp0scripts\v5brain.sh"`) do set "BRAIN=%%p"
set "RUNP="
set "ARG2=%~2"
if "%ARG2%"=="" goto :optsdone
if "%ARG2%"=="-" (
  shift
  goto :opts
)
rem a dash-prefixed 2nd arg is a robot option, not a program
if "%ARG2:~0,1%"=="-" goto :opts
if not exist "%~f2" (
  echo No such program: "%~2"
  exit /b 2
)
for /f "usebackq delims=" %%p in (`wsl -d Ubuntu-24.04 -- wslpath -a "%~f2"`) do set "RUNP=%%p"
shift
:opts
rem drop the preset; everything left is extra robot options
set "EXTRA="
shift
:more
if "%~1"=="" goto :mode
set "EXTRA=%EXTRA% %~1"
shift
goto :more
:optsdone
set "EXTRA="
:mode
set "IDOPT=--id auto"
if defined V5SIM_ID set "IDOPT=--id %V5SIM_ID%"
set "USBOPT=--usb"
if exist "%ProgramFiles%\USBip\usbip.exe" set "USBOPT=--usbip"
rem V5SIM_NO_USBIP=1 forces the WSL pty bridge instead of the Windows USB/IP
rem attach (no usbip-win2 driver involvement; ports are /tmp/ttyV5sys|user).
if "%V5SIM_NO_USBIP%"=="1" set "USBOPT=--usb"
rem Single USB/IP instance only: attaching a second V5 brain while one is
rem already on Windows USB can bugcheck usbip-win2 (0.9.7.8). If one is already
rem attached, this instance uses the pty bridge instead of a second attach.
if not "%USBOPT%"=="--usbip" goto :usbipdone
rem V5SIM_FORCE_USBIP=1 skips this guard (a 2nd concurrent attach; BSOD risk).
if "%V5SIM_FORCE_USBIP%"=="1" goto :usbipdone
"%ProgramFiles%\USBip\usbip.exe" port 2>nul | findstr /i "2888:0501" >nul
if not errorlevel 1 (
  echo A V5 brain is already attached to Windows USB.
  echo usbip is single-instance on this driver; this one uses the WSL pty bridge.
  set "USBOPT=--usb"
)
:usbipdone
set "MODE=%IDOPT% %USBOPT%"
if "%V5SIM_HEADLESS%"=="1" set "MODE=%MODE% --headless"
rem share the window-watchdog token into WSL so v5brain.sh records its id
if defined V5SIM_WINDOW_TOKEN if defined WSLENV set "WSLENV=%WSLENV%:V5SIM_WINDOW_TOKEN"
if defined V5SIM_WINDOW_TOKEN if not defined WSLENV set "WSLENV=V5SIM_WINDOW_TOKEN"
echo Robot: %PRESET%%EXTRA%   (field view and USB ports: see the [v5brain] instance line)
if defined RUNP (
  wsl -d Ubuntu-24.04 -- bash "%BRAIN%" %MODE% --robot "@%PFILE%%EXTRA%" --run "%RUNP%"
) else (
  wsl -d Ubuntu-24.04 -- bash "%BRAIN%" %MODE% --robot "@%PFILE%%EXTRA%"
)
