# indianpremierleague


https://drive.google.com/file/d/1cBViknJKF6fxfDtCdUIW-N-YunSdEf0v/view?usp=sharing



21-feb
https://drive.google.com/drive/folders/11QukBbt3hPMHALn1XnRMUasSjl92l_Xz?usp=sharing




version: "3.4"

services:

  gz:
    image: px4_ros2_docker:iron_harmonic
    # runtime: nvidia
    environment:
      - TERM=xterm-256color
      - DISPLAY
      - WAYLAND_DISPLAY
      - XDG_RUNTIME_DIR
      - PULSE_SERVER
    volumes:
      - /usr/bin/docker:/usr/bin/docker
      - /var/run/docker.sock:/var/run/docker.sock
      - /tmp/.X11-unix:/tmp/.X11-unix
      - /run/user:/run/user
      - /etc/localtime:/etc/localtime:ro
    network_mode: host
    privileged: true
    command: gz sim
