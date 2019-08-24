## Web Joystick using Flask server and roslibjs
Dependencies:
* Python: tornado, mongo-python-driver, flask, [flaskcompress](https://github.com/colour-science/flask-compress)
* ROS - rosauth, [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite)

Link to roslibjs tutorials - http://wiki.ros.org/roslibjs

### Using the package
* Build this catkin workspace and run the ```rosbridge_websocket.launch``` of ```rosbridge_server``` package
* Run the webserver by executing ```joy_gui/main.py```
