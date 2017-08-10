@echo off
set argc=0
for %%x in (%*) do Set /A argc+=1
IF %argc% LSS 2 (
   echo %0 [ptz camera IP] [xport IP]
   pause
   exit
)
echo "start program"
@echo on
start C:\nginx\sbin\nginx.exe
sleep 1
start OccamVideoServer.exe 60 verbal
sleep 1
start PTZVideoServer.exe %1
sleep 1
start node websocket-occam.js secret1
sleep 1
start node websocket-ptz.js secret2
sleep 1
start OccamVideoNode verbal
sleep 1
start PTZVideoNode verbal
sleep 1
start ptz_comm_server %2
sleep 1
start node server.js
sleep 1