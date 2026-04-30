# General information
## ros2 version
Jazzy

## Raspberry Pi
Username: robotised-demining  
Password: demining  
IP: 192.168.10.2

# Installation
Add the code inside your ros2 workspace inside the src folder

If you clone the reposatory directly into the ros2_ws folder, you file structure will have the following structure:

```
/ros2_ws/src
└── Robotised_Demining_ros2
    ├── controller_receiver   
    ├── dig_mode    
    ├── drive_mode    
    ├── mode_selector    
    ├── README.md    
    └── rover_launch
```

In this case the command:
```colcon build --base-paths src/Robotised_Demining_ros2```

Should be used to build the packages. This will point ros2 to the correct folder

## Common issues
It may be neccesary to remove the build, install, and log folders by using the command:
```
cd ~/ros2_ws
rm -rf build install log
```

It may also be neccesary to reconfigure the ros2 enviroment:
```
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
```

# Setup
The raspberry pi is configured with ubuntu server and a static ip ```192.169.10.2``` to connect your laptop to the raspberry pi connect a ethernet cable to the raspbeery pi and set your ip address to ```192.169.10.1``` with subnet mask of ```255.255.255.0``` 

Now ssh can be used to connect to the raspberry pi.
In ubuntu this can be done with the following command:
```ssh robotised-demining@192.168.10.2```
and than entering the password of the raspberry pi 

## Setup WIFI
If its neccesary to connect the raspberry pi to WIFI, the eassiest way is to activate a hotspot an your mobile phone and opening:
```
sudo nano /etc/netplan/01-netcfg.yaml
```

In a terminal connected to the raspberry pi. You will see something as the following: 
```
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 192.168.10.2/24
      optional: true

  wifis:
    wlan0:
      dhcp4: yes
      access-points:
        "YOUR_WIFI_NAME":
          password: "YOUR_PASSWORD"
```
Now enter the hotspot name and password in the appropiate places and save the document and enter:
```
sudo netplan apply
```
In the same terminal.  

It may be neccesary to reboot the raspberry pi after applying these changes using ```sudo reboot```

Note: the raspberry pi is located in a metal electronics box so wifi may not work when the electronics box is closed

# Setup vs code over ssh
To connect visual studio code to the raspberry pi first install the ssh extension in vs code 

Then open vs code and press ```CTRL + SHIFT + P``` then enter ```Remote-SSH: Connect to Host...```  
Select: ```Add new SSH Host...```  
Enter: ```ssh robotised-demining@192.168.10.2```  
Save configuration  
Enter password of raspberry pi 

vs code will save the configuration so next time will only be neccesary to:  
press ```CTRL + SHIFT + P``` then enter ```Remote-SSH: Connect to Host...```  
Select: ```192.168.10.1```  
Enter password of the raspberry pi 

Follow this tutorial for more detail: https://code.visualstudio.com/docs/remote/ssh

# Usage
## Launch all the ros2 nodes
usefull for testing
```ros2 launch rover_launch total.launch.py```

## Launch pi nodes
```ros2 launch rover_launch pi.launch.py```

## Launch pc nodes
```ros2 launch rover_launch pc.launch.py```
