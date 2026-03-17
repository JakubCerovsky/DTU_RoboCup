#!/bin/bash
# echo -e "\nStarting mission\n"
cd /home/local/svn/robobot/mqtt_python

#Tereza
# /usr/bin/python3 mission-run-tereza.py -n >>log_out_tereza.txt 2>>log_err_tereza.txt &
#/usr/bin/python3 testing_tereza.py -n >>log_out_tereza.txt 2>>log_err_tereza.txt &


# Michele - Ball test
# /usr/bin/python3 camera_test.py
#/usr/bin/python3 camera_original_orange.py

# proximity sensors
# /usr/bin/python3 proximity_sensor_test.py

# find the gate
# /usr/bin/python3 gate_test.py

# find the hole
# /usr/bin/python3 hole_test.py

# reconise suitcase
# /usr/bin/python3 Live_Aruco_Detection.py

# Half way competition
/usr/bin/python3 half_way_competition.py -n >>log_out_half.txt 2>>log_err_half.txt &

# Original
# /usr/bin/python3 mqtt-client.py -n >>log_out.txt 2>>log_err.txt &

exit 0
