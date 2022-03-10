# build the container
# >> docker build -f [filename.dockerfile] -t [NAME] .
DOCKER_BUILDKIT=1 docker build -f Dockerfile -t frogs_fm_gripper_module .

