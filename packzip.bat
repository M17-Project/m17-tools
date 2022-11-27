cd D:/bin
FOR /F %%i IN ('where libcodec2.dll') DO FOR /F %%a IN ('dumpbin /DEPENDENTS m17-mod.exe ^| findstr .*dll ^| findstr /V K.') DO ECHO F|xcopy %%i\..\%%a .\%%a