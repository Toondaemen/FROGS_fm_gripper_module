# s is an auto generated Dockerfile for ros:desktop-full
# generated from docker_images/create_ros_image.Dockerfile.em
#FROM osrf/ros:kinetic-desktop-xenial

# install ros packages
#RUN apt update && apt-get update && apt-get install -y --no-install-recommends \
#    ros-kinetic-desktop-full=1.3.2-0* \
#    && rm -rf /var/lib/apt/lists/*

#RUN apt update && apt -y install vim && \
#        apt -y install tmux && \  
#        apt -y install git && \
#        apt -y install sudo && \
#        apt -y install net-tools && \
#        apt-get install -y python3-pip && \
#        apt-get -y install python-rdflib wget xterm


#RUN curl "https://bootstrap.pypa.io/get-pip.py" -o "get-pip.py"
#RUN curl "https://bootstrap.pypa.io/pip/2.7/get-pip.py" -o "get-pip.py"

#RUN python get-pip.py
#RUN pip install wrapt inflection pymodbus

#RUN apt install ros-kinetic-soem

#RUN pip3 install pandas requests seaborn pyserial




# use official ros kinetic image
FROM osrf/ros:melodic-desktop-full

# install ros packages
RUN apt update \
&&  apt-get update \
&&  apt-get install -y --no-install-recommends ros-melodic-desktop-full \
&&  rm -rf /var/lib/apt/lists/*

RUN apt update && apt -y install vim \
&&  apt -y install tmux \  
&&  apt -y install git \
&&  apt -y install sudo \
&&  apt -y install net-tools \
&&  apt-get -y install python-rdflib wget xterm \
&&  apt-get -y install lsb-release lsb-core usbutils gdebi python-pip unzip \
&&	apt-get -y install ros-melodic-soem \
&&	apt-get -y install ros-melodic-rosserial-arduino \
&&  apt-get -y install ros-melodic-rosserial
	    

# for pip (if needed)
#RUN curl "https://bootstrap.pypa.io/get-pip.py" -o "get-pip.py"
RUN curl "https://bootstrap.pypa.io/pip/2.7/get-pip.py" -o "get-pip.py"
RUN python get-pip.py
RUN pip install wrapt inflection pymodbus

# install python3 pckgs
RUN apt-get update \ 
&&  apt-get install -y python3.7 \
&&  apt-get install -y python3.7-dev \
&&  apt-get install -y python3.7-venv \
&&  apt-get install -y python3-pip \
&&  apt-get install -y python3-catkin-pkg-modules \
&&  apt-get install -y python3-rospkg-modules \
&&  apt-get install -y python-tk 

RUN pip3 install pandas requests seaborn pyserial

# setup entrypoint

#RUN bash -c "chmod +x /ros_entrypoint.sh"
#ENTRYPOINT ["/ros_entrypoint.sh"]

# Define a user to execute the script
ARG UNAME=ros
ARG UID=1001
ARG GID=1001
ENV UNAME ${UNAME}
RUN groupadd -g $GID $UNAME
RUN useradd -m -u $UID -g $GID -s /bin/bash $UNAME
RUN echo "$UNAME:ros" | chpasswd && adduser $UNAME sudo
USER $UNAME

#COPY ./sudoers /etc/sudoers

# This will the working directory of the container
WORKDIR /home/ros/

EXPOSE 11011-11310 
# Forward the roscore port
EXPOSE 11311
# forward browser port

