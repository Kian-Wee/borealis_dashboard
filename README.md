# Edited from kian wee's git hub 
https://github.com/Kian-Wee/borealis_dashboard
# Dependencies 
- mt_msgs (dso custom msgs it is inside src folder)
- pythonping (not sure if this exist for python 2.7x)
https://pypi.org/project/pythonping/
- re (regex, inside default python lib)

from mt_msgs.msg import phaseAndTime
from pythonping import ping 
import re

# Notes
Build the mt msgs package first
Then apt-get install python ping
Then build the package
It shoudl build successfully hopefully

Changes made are in a new paragraph, quite seperated from the rest of the code.
Changes made are only in the following scripts 
- uav_status_visualizer.py
- dashboard.py
Throw away changes