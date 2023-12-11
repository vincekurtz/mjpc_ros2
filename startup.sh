#!/bin/bash
join=false
while getopts n:jc flag
do
    case "${flag}" in
        n) image_name_or_id=${OPTARG};;
        j) join=true;;
    esac
done
xhost +local:docker
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    if [ "$join" = false ] ; then

        # checks whether system has GPU
        if (($(lspci | grep -ci vga) > 1)); then
            docker run --rm -it --net=host --ipc=host -e DISPLAY=$DISPLAY \
            -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            -e NVIDIA_DRIVER_CAPABILITIES=all \
            -v $HOME/.Xauthority:/root/.Xauthority:rw \
            -v $PWD/dev_ws/src/nerf_custom/collect_data/data:/root/dev_ws/src/nerf_custom/collect_data/data \
            --privileged --runtime=nvidia --gpus all --cap-add=NET_ADMIN \
            --name=$image_name_or_id \
            $image_name_or_id

        else
            docker run --rm -it --net=host --ipc=host -e DISPLAY=$DISPLAY \
            -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
            -e NVIDIA_DRIVER_CAPABILITIES=all \
            -v $HOME/.Xauthority:/root/.Xauthority:rw --cap-add=NET_ADMIN \
            -v $PWD/dev_ws/src/nerf_custom/collect_data/data:/root/dev_ws/src/nerf_custom/collect_data/data \
            --privileged \
            --name=$image_name_or_id \
            $image_name_or_id
        fi
    else
        # join a running container
        docker exec -it -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
        $image_name_or_id /bin/bash
    fi
fi
xhost -local:docker
