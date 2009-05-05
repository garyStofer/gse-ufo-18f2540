@echo off
rem ---------------------------------------------
rem Delete all working files but not hex files
rem ---------------------------------------------

set CSRC= adc accel c-ufo irq lisl main pid sensor serial utils globals prog
set ASRC=bootl18F2520 general

rem compiler working files...
for %%i in ( %CSRC% ) do if exist %%i.asm del %%i.asm
for %%i in ( %CSRC% ) do if exist %%i.err del %%i.err
for %%i in ( %CSRC% ) do if exist %%i.fcs del %%i.fcs
for %%i in ( %CSRC% ) do if exist %%i.lst del %%i.lst
for %%i in ( %CSRC% ) do if exist %%i.o   del %%i.o  
for %%i in ( %CSRC% ) do if exist %%i.occ del %%i.occ
for %%i in ( %CSRC% ) do if exist %%i.var del %%i.var

rem assembler working files...
for %%i in ( %ASRC% ) do if exist %%i.err del %%i.err
for %%i in ( %ASRC% ) do if exist %%i.lst del %%i.lst
for %%i in ( %ASRC% ) do if exist %%i.o   del %%i.o

rem linker working files...
if exist *.cod del *.cod
if exist *.lst del *.lst
if exist *.map del *.map
if exist *.cof del *.cof
if exist *.hex del *.hex
