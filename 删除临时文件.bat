::@echo off 

@for /d /r %%c in (Debug,settings) do @if exist %%c ( rd /s /q "%%c" & echo     删除目录%%c) 

@for /r  %%c in (*.tmp,*.bak ,*.dep,*.sfr,Backup*,*.tmp.c ) do del "%%c"
:: *.ewd 是下载配置文件
pause