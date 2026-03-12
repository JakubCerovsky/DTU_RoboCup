# RoboBot Development Guide

This guide describes the **recommended workflow for developing and
deploying code on the RoboBot**. While there are multiple ways to
interact with the robot, the method described here ensures **consistent
collaboration within the team**.

---

# 1. Prerequisites

Before starting, make sure the following tools are installed:

## Required Software

- **Visual Studio Code**\
  https://code.visualstudio.com/

- **Remote Explorer Extension**\
  https://marketplace.visualstudio.com/items?itemName=ms-vscode.remote-explorer

- **Remote -- SSH Extension**\
  https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-ssh

## Optional (Recommended)

- **WinSCP** or similar file transfer tool\
  Used for downloading robot files locally.

---

# 2. Accessing the Robot

Follow these steps to connect to the robot using **VS Code Remote SSH**.

## Step 1 --- Open Remote Explorer

1.  Open **Visual Studio Code**
2.  Click the **Remote Explorer** icon in the left sidebar
3.  Select **Remotes (Tunnels/SSH)** from the dropdown at the top

---

## Step 2 --- Add a New Remote Host

1.  Click **Add New Remote**
2.  Give the robot a name (example: **June the Robot**)

---

## Step 3 --- Configure SSH

1.  Open the **SSH tab**
2.  Click the **Settings (⚙)** icon
3.  Select **Open SSH Config File**
4.  Choose the **first configuration file**

Add the following entry:

    Host June the Robot
      HostName <ROBOT_IP_ADDRESS>
      User local

Example:

    Host June the Robot
      HostName 192.168.0.55
      User local

---

## Step 4 --- Connect to the Robot

1.  In **Remote Explorer**, select **June the Robot**
2.  Click **Connect in New Window**

VS Code will open a **remote development session directly on the
robot**.

---

## Step 5 --- Login

When prompted, enter the password:

    grenen

You are now **working directly on the robot filesystem**.

⚠️ Important: All file changes happen **live on the robot**.

---

# 3. Development Workflow

Developers should create or modify Python files in:

    /home/local/svn/robobot/mqtt_python

Example structure:

    robobot/
     ├── mqtt_python/
     │   ├── mqtt-client.py
     │   ├── navigation.py
     │   └── example_script.py
     ├── mission_start.bash
     └── on_reboot.bash

---

# 4. Running Your Python Script

Robot scripts are launched through the **mission_start.bash** script.

Location:

    /home/local/mission_start.bash

## Default Script

    cd /home/local/svn/robobot/mqtt_python
    /usr/bin/python3 mqtt-client.py -n >>log_out.txt 2>>log_err.txt &

---

## Running Your Own Script

Edit `mission_start.bash` and replace the script name:

    cd /home/local/svn/robobot/mqtt_python
    /usr/bin/python3 <YOUR_SCRIPT_NAME>.py

Example:

    cd /home/local/svn/robobot/mqtt_python
    /usr/bin/python3 navigation_test.py

Optional logging version:

    /usr/bin/python3 navigation_test.py >>log_out.txt 2>>log_err.txt &

---

# 5. Starting the Robot

Once your script is configured:

1.  Save `mission_start.bash`
2.  Go to the robot
3.  Press the **GREEN START BUTTON** on top of the robot

Your script will now execute.

---

# 6. Viewing Logs

Check output logs:

    cat log_out.txt

Live log monitoring:

    tail -f log_out.txt

Error logs:

    cat log_err.txt

---

# 7. Saving Changes Locally

Download the **robobot folder** from the robot.

Using **WinSCP**, navigate to:

    /home/local/svn/robobot

Save it locally inside:

    DTU_RoboCup/
    └── svn/
      └── robobot/
        └── mqtt_python/
    ├── mission_start.bash
    └── on_reboot.bash

---

# 8. Uploading Changes to GitHub

Navigate to the **DTU_RoboCup repository folder**.

### Stage changes

    git add -A

### Commit

    git commit -m "Describe your changes"

### Push

    git push

---

# 9. Best Practices

## Naming

Use descriptive file names:

    navigation_controller.py
    mqtt_listener.py
    obstacle_detection.py

Avoid:

    test.py
    script1.py

## Commit Messages

Good examples:

    Fix MQTT reconnect bug
    Add lidar obstacle detection
    Improve navigation PID tuning

Bad examples:

    fix
    update
    stuff

---

# 10. Common Issues

## SSH Connection Fails

Check:

- Robot IP address
- Network connection
- SSH config formatting

## Python Script Does Not Run

Verify:

    python3 <script>.py

Also check:

    log_err.txt

## Script Stops Immediately

Possible causes:

- Missing Python packages
- Syntax errors
- Incorrect file path
