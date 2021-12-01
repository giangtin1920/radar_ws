@echo off

set MATLAB=A:\R2020b

cd .

if "%1"=="" ("A:\R2020b\bin\win64\gmake"  -f testCoder_rtw.mk all) else ("A:\R2020b\bin\win64\gmake"  -f testCoder_rtw.mk %1)
@if errorlevel 1 goto error_exit

exit 0

:error_exit
echo The make command returned an error of %errorlevel%
exit 1