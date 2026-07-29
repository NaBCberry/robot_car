@echo off
setlocal EnableExtensions
py -3 "%~dp0capture_protocol_frames.py" %*
if errorlevel 9009 python "%~dp0capture_protocol_frames.py" %*
