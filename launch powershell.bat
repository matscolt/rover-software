@echo off
cd /d %~dp0
start powershell -NoExit -Command "Set-Location -LiteralPath '%cd%'"
