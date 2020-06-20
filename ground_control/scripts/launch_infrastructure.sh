gnome-terminal -e "mavproxy.py --master=udp:127.0.0.1:14550" &
cd ../../qgroundcontrol/
gnome-terminal -- "./QGroundControl.AppImage" &

