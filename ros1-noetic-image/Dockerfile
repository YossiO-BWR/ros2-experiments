ARG BASE_VERSION=ros:noetic
FROM $BASE_VERSION AS builder

ENV DEBIAN_FRONTEND=noninteractive

# USE BASH
SHELL ["/bin/bash", "-c"]

RUN mkdir -p /ws/src

WORKDIR /ws
RUN mkdir -p /ws/src/yo_exp1
RUN apt update && apt install -y build-essential gcc cmake make python3-catkin-tools tmux htop
RUN source /opt/ros/noetic/setup.bash && catkin init && catkin config --extend /opt/ros/noetic

WORKDIR /ws/src
COPY yo_exp1 /ws/src/yo_exp1

WORKDIR /ws/
RUN catkin build --summary --no-status --continue-on-failure
RUN echo "source /ws/devel/setup.bash" >> /etc/bash.bashrc
RUN echo "source /ws/devel/setup.bash" >> ~/.bashrc
RUN echo "alias launch='roslaunch /ws/src/yo_exp1/launch'" >> ~/.bashrc
RUN echo "alias bw='rostopic bw -w10 /node/topic_000'" >> ~/.bashrc

CMD ["roscore"]
