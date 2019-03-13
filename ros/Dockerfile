FROM ros:melodic

RUN apt-get update && apt-get upgrade -y 
RUN apt-get install -y nano iputils-ping
RUN apt-get install -y ros-melodic-rosserial ros-melodic-rosserial-server ros-melodic-rosserial-python
RUN apt-get clean && apt-get autoremove -y

RUN echo source /opt/ros/melodic/setup.bash > .bashrc

CMD ["rosrun", "rosserial_python", "serial_node.py", "tcp"]
