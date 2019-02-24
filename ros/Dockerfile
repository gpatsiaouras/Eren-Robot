FROM ros:melodic-ros-core

RUN apt-get update && apt-get upgrade -y 
RUN apt-get install -y ros-melodic-rosserial ros-melodic-rosserial-server ros-melodic-rosserial-python
RUN apt-get clean && apt-get autoremove -y

CMD ["roslaunch", "rosserial_server", "socket.launch"]