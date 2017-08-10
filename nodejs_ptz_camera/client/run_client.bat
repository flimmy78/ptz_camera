@echo off
set argc=0
for %%x in (%*) do Set /A argc+=1
IF %argc% LSS 1 (
   echo %0 [server IP]
   pause
   exit
)
echo "start client"
@echo on
node client.js %1
