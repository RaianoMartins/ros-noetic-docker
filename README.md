# ROS Noetic Docker Image

![Docker](https://img.shields.io/badge/Docker-Ready-blue?logo=docker)
![ROS](https://img.shields.io/badge/ROS-Noetic-blue?logo=ros)
![Ubuntu](https://img.shields.io/badge/Ubuntu-20.04-orange?logo=ubuntu)
![License](https://img.shields.io/badge/License-MIT-green)

---

# Table of Contents
- [Overview](#-overview)
- [1. Build the Image](#1-build-the-image)
- [2. First Run](#2-first-run)
- [3. Reattaching / Restarting](#3-reattaching--restarting)
- [4. Saving Container Modifications](#4-saving-container-modifications)
- [5. Useful Docker Commands](#5-useful-docker-commands)
  - [Container Management](#container-management)
  - [Image Management](#image-management)
  - [Volume Management](#volume-management)
  - [Network Management](#network-management)
  - [Development & Debugging](#development--debugging)
- [Contributions](#-contributions)

---

# Overview

This repository provides a **ready-to-use Docker image for ROS Noetic**, including:

- GUI (X11) support  
- Persistent development workspace  
- Non-root user with passwordless sudo  
- Device access (for serial, sensors, etc.)  
- Pre-installed ROS development tools  

---

# 1. Build the Image

Inside the `image/` folder, run:

```bash
docker build -t ros-noetic-image .
```

---

# 2. First Run

```bash
docker run -it --name ros-noetic-container --network=host   --ipc=host   -v ~/docker_volumes/ros-noetic/catkin_ws:/home/ros-noetic/catkin_ws   -v ~/docker_volumes/ros-noetic/config:/home/ros-noetic/.config/ros   -v /tmp/.X11-unix:/tmp/.X11-unix:rw   -v ~/.ssh:/home/ros-noetic/.ssh:ro   --env=DISPLAY   -v /dev:/dev   --device-cgroup-rule='c *:* rmw'   ros-noetic-image
```

Includes:

- Persistent catkin workspace  
- X11 GUI support  
- Read-only SSH mounting  
- `/dev` access  
- Host networking (useful for ROS1)  

---

# 3. Reattaching / Restarting

### Detach without stopping  
```
Ctrl + P, Ctrl + Q
```

### Reattach  
```bash
docker attach ros-noetic-container
```

### Stop
```bash
docker stop ros-noetic-container
```

### Start again
```bash
docker start ros-noetic-container
```

### Open a new shell
```bash
docker exec -it ros-noetic-container bash
```

---

# 4. Saving Container Modifications

To persist manual installs or changes:

```bash
docker commit ros-noetic-container ros-noetic-container-custom
```

---

# 5. Useful Docker Commands

---

## Container Management

```bash
docker run -it image_name
docker run -d --name mycontainer image_name
docker run -p 8080:80 image_name
docker run -v /host/path:/container/path image_name
docker run -e VAR=value image_name
docker run --rm image_name
docker run --privileged image_name

docker start container_name
docker stop container_name
docker restart container_name
docker pause container_name
docker unpause container_name
docker rm container_name
docker rm -f container_name

docker ps
docker ps -a
docker logs container_name
docker logs -f container_name
docker inspect container_name
docker top container_name
docker stats container_name
docker stats
docker diff container_name
docker port container_name
```

---

## Image Management

```bash
docker images
docker pull image_name:tag
docker build -t image_name .
docker rmi image_name

docker image prune
docker image inspect image_name
docker history image_name
docker save -o image.tar image_name
docker load -i image.tar
docker tag image_name new_name
```

---

## Volume Management

```bash
docker volume ls
docker volume create volume_name
docker volume inspect volume_name
docker volume rm volume_name
docker volume prune
```

---

## Network Management

```bash
docker network ls
docker network create network_name
docker network connect network_name container_name
docker network disconnect network_name container_name
docker network inspect network_name
docker network rm network_name
```

---

## Development & Debugging

```bash
docker exec -it container_name bash
docker exec -it --user user container_name bash

docker cp file.txt container_name:/path/
docker cp container_name:/path/file.txt ./

docker commit container_name new_image
docker rename old_name new_name
```

---

# Contributions

Pull requests are welcome!  
Feel free to contribute improvements to the Dockerfile, workspace system, or documentation.

---
