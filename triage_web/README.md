# DARPA_triage2026
=======
Requirements

Ubuntu 22.04

ROS 2 Humble

Python 3.10

A working camera (or webcam)

--------------------------------

How to Run (Local)

You need two terminals.
Terminal 1 — Start the Camera (ROS)
    source /opt/ros/humble/setup.zsh
    ros2 run image_tools cam2image


Terminal 2 - Start the web Backend 

    source /opt/ros/humble/setup.zsh
    cd ~/triage_web/backend
    python3 app.py


OPEN ON http://localhost:8080


****WORKING ENDPOINTS 
Available Endpoints
Endpoint	Description
/	            Web dashboard
/image	        Latest camera frame (JPEG)
/location	    ROS-based triage location
/ip_location	IP-based operator location
>>>>>>> 19f6686 (Initial DARPA Triage 2026 web GUI with ROS + Flask)
