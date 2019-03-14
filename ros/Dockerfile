FROM ros:melodic

RUN apt-get update && apt-get upgrade -y 
RUN apt-get install -y nano iputils-ping
RUN apt-get install -y ros-melodic-rosserial ros-melodic-rosserial-server ros-melodic-rosserial-python
RUN apt-get clean && apt-get autoremove -y

RUN echo "roscore &>/dev/null &" > /init_roscore_and_server.sh
RUN echo "sleep 3s" >> /init_roscore_and_server.sh
RUN echo "rosrun rosserial_python serial_node.py tcp" >> /init_roscore_and_server.sh

CMD ["sh", "/init_roscore_and_server.sh"]
