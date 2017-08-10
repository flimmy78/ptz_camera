sleep 1
@taskkill /im OccamVideoServer.exe /f
sleep 1
@taskkill /im PTZVideoServer.exe /f
sleep 1
@taskkill /im ffmpeg.exe /f
sleep 1
@taskkill /im OccamVideoNode.exe /f
sleep 1
@taskkill /im PTZVideoNode.exe /f
sleep 1
@taskkill /im ptz_comm_server.exe /f
sleep 1
@taskkill /im node.exe /f