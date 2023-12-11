A basic example of integrating MJPC with a ROS2 pipeline.

## Instructions

to build the image (every time you update a local file)
```
docker image build -t mjpc -f Dockerfile.mjpc .
```

to start a container with a built image
```
bash startup.sh -n mjpc
```

to join an existing container
```
bash startup.sh -n mjpc -j
```

