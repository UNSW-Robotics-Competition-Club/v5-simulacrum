@echo off
rem pros upload, letting you choose the brain when several are on USB
rem (simulators are named by instance). Run it in a PROS project folder:
rem
rem   V5Upload [path] [pros upload options...]
rem
rem   e.g.  V5Upload --slot 2 --after run
rem
rem pros-cli's own prompt for several brains crashes on Windows, so this asks
rem instead and runs  pros upload <path or .> COMn [options...]
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0scripts\v5upload.ps1" %*
