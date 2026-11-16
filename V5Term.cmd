@echo off
rem pros terminal on the brain you choose when several are on USB
rem (simulators are named by instance):
rem
rem   V5Term [pros terminal options...]
rem
rem Plain pros terminal finds no port at all when Windows has more than one
rem V5 device, so this asks and runs  pros terminal COMn [options...]
rem with that device's User port. Open it before starting the program to
rem catch the PROS banner.
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\v5term.ps1" %*
