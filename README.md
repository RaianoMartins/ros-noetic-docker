# ROS-Noetic docker image


## This repository was created to host an easy and ready-to-use ROS Noetic Docker image. Feel free to modify it and make improvements.


### INSTRUCTIONS


1) Running it for the very first time.


In image folder, use the following command to build.


```
docker build -t ros-noetic-image .
```

When it's finished, run the container.


```
docker run -it --name ros-noetic-container \
  --user ros-noetic \
  --network=host \
  --ipc=host \
  -v ~/docker_volumes/ros-noetic/catkin_ws:/home/ros-noetic/catkin_ws \
  -v ~/docker_volumes/ros-noetic/config:/home/ros-noetic/.config/ros \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ~/.ssh:/home/ros-noetic/.ssh:ro \
  --env=DISPLAY \
  -v /dev:/dev \
  --device-cgroup-rule='c *:* rmw' \
  ros-noetic-image
```


2) Re initializing the container


Exiting while keep container running.


Type Ctrl + P and Ctrl + Q


To back for the running container.


```
docker attach ros-noetic-container
```

Closing and start the container again


Inside it, type.


```
exit
```

Or outside, type.


```
docker stop ros-noetic-container
```

To start the container again.


```
docker start ros-noetic-container
```

To run it again.


```
docker exec -it ros-noetic-container bash
```

3) Update image for persist important changes (e.g. manually installed packages).


```
docker commit ros-noetic-container ros-noetic-container-custom
```

4) Useful commands.


#### CONTAINER MANAGEMENT


docker run -it image_name                              # Run interactive container
docker run -d --name mycontainer image_name            # Run detached container
docker run -p 8080:80 image_name                       # Run with port mapping
docker run -v /host/path:/container/path image_name    # Run with volume
docker run -e VAR=value image_name                     # Run with environment variable
docker run --rm image_name                             # Auto-remove after exit
docker run --privileged image_name                     # Run with privileges
docker start container_name                            # Start stopped container
docker stop container_name                             # Stop running container
docker restart container_name                          # Restart container
docker pause container_name                            # Pause container
docker unpause container_name                          # Unpause container
docker rm container_name                               # Remove container
docker rm -f container_name                            # Force remove container

docker ps                                              # List running containers
docker ps -a                                           # List all containers
docker logs container_name                             # Show container logs
docker logs -f container_name                          # Follow logs in real-time
docker inspect container_name                          # Inspect container details
docker top container_name                              # Show container processes
docker stats container_name                            # Show container resource usage
docker stats                                           # Show all containers stats
docker diff container_name                             # Show filesystem changes
docker port container_name                             # Show port mappings


#### IMAGE MANAGEMENT    


docker images                                          # List all images
docker pull image_name:tag                             # Download image
docker build -t image_name .                           # Build image from Dockerfile
docker rmi image_name                                  # Remove image
docker image prune                                     # Remove unused images
docker image inspect image_name                        # Inspect image details
docker history image_name                              # Show image layers
docker save -o image.tar image_name                    # Save image to tar file
docker load -i image.tar                               # Load image from tar file
docker tag image_name new_name                         # Tag an image


#### VOLUME MANAGEMENT   


docker volume ls                                       # List all volumes
docker volume create volume_name                       # Create volume
docker volume inspect volume_name                      # Inspect volume
docker volume rm volume_name                           # Remove volume
docker volume prune                                    # Remove unused volumes


#### NETWORK MANAGEMENT  


docker network ls                                      # List all networks
docker network create network_name                     # Create network
docker network connect network_name container_name     # Connect container to network
docker network disconnect network_name container_name  # Disconnect container
docker network inspect network_name                    # Inspect network
docker network rm network_name                         # Remove network


#### DEVELOPMENT & DEBUGGING


docker exec -it container_name bash                    # Execute command in running container
docker exec -it --user user container_name bash        # Execute as specific user
docker cp file.txt container_name:/path/               # Copy file to container
docker cp container_name:/path/file.txt ./             # Copy file from container
docker commit container_name new_image                 # Create image from container changes
docker rename old_name new_name                        # Rename container