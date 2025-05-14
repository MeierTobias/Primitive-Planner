FROM osrf/ros:noetic-desktop-full

ARG USER_ID
ARG GROUP_ID

# install dependencies
RUN apt update && apt upgrade -y
RUN apt install -y python3-pip python3-tk
RUN pip3 install numpy --upgrade
RUN pip3 install toppra catkin_pkg PyYAML empy matplotlib pyrfc3339 

# create a user matching the host
RUN groupadd --gid ${GROUP_ID} hostuser \
    && useradd --uid ${USER_ID} --gid ${GROUP_ID} --create-home --shell /bin/bash hostuser

RUN apt install -y nano
    
# Add line to bashrc
RUN echo "\n if [ -f ~/primitive-planner/devel/setup.bash ]; then\n source ~/primitive-planner/devel/setup.bash\n fi" >> /home/hostuser/.bashrc \
    && chown hostuser:hostuser /home/hostuser/.bashrc

ENV HOME=/home/hostuser
WORKDIR /home/hostuser
USER hostuser

CMD ["bash"]
