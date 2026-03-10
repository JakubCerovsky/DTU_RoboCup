#!/bin/bash
# echo -e "\nStarting mission\n"

cd /home/local/svn/robobot/mqtt_python
echo "$(date '+%Y-%m-%d %H:%M:%S') START_BUTTON mission-run-tereza" >>log_out_tereza.txt
/usr/bin/python3 mission-run-tereza.py -n >>log_out_tereza.txt 2>>log_err_tereza.txt &
# echo "mission ended"
exit 0
