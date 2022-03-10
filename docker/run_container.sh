#docker run --rm --name="frogs_fm_gripper_driver" -it -v $(pwd):/home/ros --network="host" --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" --device=/dev/ttyUSB0 --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" frogs_fm_gripper bash

docker run -it \
           --rm \
           -v "$(pwd)/..":/home/ros \
           --net="host" \
           --env="DISPLAY" \
           --env="QT_X11_NO_MITSHM=1" \
           --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
           --privileged -v /dev/bus/usb:/dev/bus/usb \
           --privileged -v /dev/ttyACM0:/dev/ttyACM0 \
           --name="frogs_fm_gripper_module" \
           frogs_fm_gripper_module bash
