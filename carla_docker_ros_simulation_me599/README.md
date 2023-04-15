# Documentation 

This file will provide a top level summary of everything you will need to setup this project, along with notes on the components used. 

[1.0 Software & Tools](#sw-tools)

[2.0 Deployment](#deploy) 

[3.0 Development](#develop)

# 1.0 Software & Tools <a name="sw-tools"></a>

Below is a summary of all the software and tooling used for this project.

## 1.1 Docker 

### 1.1.1 Summary 

Docker is a popular platform for developing, packaging, and deploying software applications in a containerized environment. 
A Docker container is a lightweight, standalone executable package that includes all the necessary components, such as libraries,
dependencies, and configuration files, required to run an application. Further, Docker allows developers to create, share, and deploy 
containers easily and consistently across different environments, such as local machines, cloud servers, and data centers. 
It provides benefits such as improved efficiency, portability, and scalability, making it a valuable tool for modern software development and deployment.

The [Getting Started](https://docs.docker.com/get-started/) from Docker provides a good walk-through of how to use Docker.

You may use the Docker CLI or the Docker Desktop application, however for this project the Docker CLI will be used.

The following [Docker Cheatsheet](https://docs.docker.com/get-started/docker_cheatsheet.pdf) provides a great summary of the 
primary docker CLI commands. 

Use can also always type: `docker --help` to get a summary of commands in the CLI. 

### 1.1.2 Setup & Install

**Ensure you have installed Docker**
[Ubuntu Install Guide](https://docs.docker.com/engine/install/ubuntu/)

Import additional steps for ubuntu install: 
[Linux Additional Install Steps](https://docs.docker.com/engine/install/linux-postinstall/)

> A script to install docker & docker compose is available under `utils/docker_linux_install.sh` 

## 1.2 Docker-compose 

### 1.2.1 Summary

Docker Compose is a tool that enables developers to define and run multi-container Docker applications in a declarative manner. 
Compose allows users to describe their application as a set of services, each of which is defined in a separate Dockerfile. 
Services can be linked together and configured via a Compose file, which specifies the containers, networks, and volumes required to run the application.

The following section [Use Docker Compose](https://docs.docker.com/get-started/08_using_compose/) in the getting started guide provides a overview of Docker Compose.

The following [Docker Compose Cheatsheet](https://devhints.io/docker-compose) provides a good summary. 

Use can also always type: `docker-compose --help` to get a summary of commands in the CLI. 

### 1.2.2 Setup & Install

**Ensure you have installed Docker Compose**
[Docker Compose Install Guide](https://www.digitalocean.com/community/tutorials/how-to-install-and-use-docker-compose-on-ubuntu-22-04)

> A script to install docker & docker compose is available under `utils/docker_linux_install.sh` 

**Note**: A quick note, when running docker-compose, if you run the standard command `docker-compose up` this will run all the containers
in the terminal, you will be able to see all the logs from each one. Typically, running in detached mode by specifying the `-d` flag is preferable
as this runs all the containers in the background, you can see them running with `docker ps`. 

## 1.3 ROS

### 1.3.1 Summary

ROS, which stands for Robot Operating System, is an open-source framework for building and programming robots. 
It provides a set of libraries and tools that facilitate the development of complex robotics applications, 
including communication between software modules, control of hardware components, and integration of sensors and actuators.

In particular, ROS 2 will be utilized (it will be refered to as ROS, however note that this guide focuses on ROS 2);
a list of the available [Distributions](https://docs.ros.org/en/rolling/Releases.html). 
In general, the Foxy and Galactic will be the best choice as they are relatively recent but have had some time for support by 3rd parties. 
Overall, they should be pretty interchangable but that will need to be evaluated on a case by case basis. 

For any ROS inqueries, the [Tutorials](https://docs.ros.org/en/foxy/Tutorials.html) are pretty helpful. For this project you 
don't need to install ROS locally. 

### 1.3.2 Running ROS in Docker

You can experiment using ROS by simiply pull a docker container of a specific distribution and then run a shell within the container
to run ROS exampes. You can also use the available docker containers as a base and then docker your own custom image with the specific 
packages and configurations you desire, see `./docker/ros2_container/Dockerfile` as an example.

**Pull and Run ROS in a Docker Container**

Pull a container for the appropiate distribution: [Containers](https://hub.docker.com/r/osrf/ros/tags?page=1)
```
docker pull osrf/ros:foxy-desktop-focal
```

Run container: 
```
docker run -it <image-id>
```

Once inside you can run regular ROS 2 commands:
```
ros2 topic list
ros2 pkg list
```

Run demo nodes:
```
ros2 run demo_nodes_cpp listener & ros2 run demo_nodes_cpp talker
```

**Run ROS using Docker Compose**

Docker-compose demo:
```
docker-compose --file ros2_nodes_demo.yaml up
```

## 1.4 CARLA 

### 1.4.1 Summary

CARLA is an open-source simulator designed for testing and developing autonomous driving systems. It provides a high-fidelity simulation
environment with realistic urban and suburban scenarios, allowing developers to test various perception, planning, and control 
algorithms under different driving conditions. CARLA simulates the behavior of other traffic participants, such as pedestrians, 
cyclists, and other vehicles, making it suitable for testing advanced driving assistance systems and autonomous vehicles.

### 1.4.2 Running CARLA in Docker

It is possible to install and run Carla locally, however it is also easy to just use the available docker image. 
You can run a CARLA instance locally using docker-compose, using the example: `./scripts/carla.yaml`, by running: 

```
docker-compose --file ./scripts/carla.yaml up -d
```

This will create and start a CARLA container with the GUI visible. 

Alternatively, you can also do the following: 

Install CARLA docker container: 
```
docker pull carlasim/carla:latest
```

Launch carla:

```
docker run \
 -p 2000-2002:2000-2002 \
 --cpuset-cpus="0-5" \
 --runtime=nvidia \
 --gpus 'all,"capabilities=graphics,utility,display,video,compute"' \
 -e display=$display \
 -v /tmp/.x11-unix:/tmp/.x11-unix \
 -it \
 carlasim/carla \
 ./carlaue4.sh -vulkan $1
```

To interact with CARLA, to either spawn agents or change the weather you'll need to use the python API. 
You can create a environment locally, and then create the scripts there, such as: 

```
conda env create --name carla python=3.7
conda activate carla
pip install pygame numpy 
```

Or you can create a docker container with a confiured environment, see an example: 
> `./docker/carla_client_node/Dockerfile`

## 1.5 Foxglove

## 1.6 Linux & Other

## 1.7 Watonomous

# 2.0 Deployment <a name="deploy"></a>

## 2.1 Project Structure

# 3.1 Development <a name="develop"></a>

## Run CARLA Locally

install docker container 
```
docker pull carlasim/carla:latest
```

create python environment (using conda) 
```
conda env create --name carla python=3.7
conda activate carla
pip install pygame numpy 
```

copy over python api from container
```
docker cp <container id>:/home/carla/pythonapi <dest>
```

launch carla
```
docker run \
 -p 2000-2002:2000-2002 \
 --cpuset-cpus="0-5" \
 --runtime=nvidia \
 --gpus 'all,"capabilities=graphics,utility,display,video,compute"' \
 -e display=$display \
 -v /tmp/.x11-unix:/tmp/.x11-unix \
 -it \
 carlasim/carla \
 ./carlaue4.sh -opengl $1
```

To interact with Carla using the python API, cd in the local PythonAPI directory copied over. 
Activate the python environment created and you can run scripts from the /examples folder. 

## Run a ROS 2 Humble Container Locally

Pull a container for the appropiate distribution: [Containers](https://hub.docker.com/r/osrf/ros/tags?page=1)
```
docker pull osrf/ros:humble-desktop-full
```

Run container: 
```
docker run -it <image-id>
```

Once inside you can run regular ROS 2 commands:
```
ros2 topic list
ros2 pkg list
```

Run demo nodes:
```
ros2 run demo_nodes_cpp listener & ros2 run demo_nodes_cpp talker
```

Docker-compose demo:
```
docker-compose --file ros2_nodes_demo.yaml up
```

# Memory Usage

From `docker stats`:

```
CONTAINER ID   NAME                                         CPU %     MEM USAGE / LIMIT     MEM %     NET I/O           BLOCK I/O         PIDS
21f61acb0384   ros2_container                               0.05%     40.39MiB / 30.78GiB   0.13%     664kB / 251kB     2.36MB / 852kB    11
fb2b86da4423   foxglove_bridge                              1.34%     59.65MiB / 30.78GiB   0.19%     216MB / 125MB     20.7MB / 700kB    28
24ba8e482ca5   carla_client                                 101.56%   13.07MiB / 30.78GiB   0.04%     4.01MB / 73.3kB   8.76MB / 0B       10
de63bdd20587   carla_ros_bridge                             63.58%    238MiB / 30.78GiB     0.76%     619MB / 217MB     17.2MB / 4.51MB   51
e7f5a5cabd2f   carla_server                                 67.14%    1.98GiB / 30.78GiB    6.43%     1.31MB / 622MB    594MB / 8.19kB    57
728167940c63   sample_docker_carla_project_ros_listener_1   0.11%     21.66MiB / 30.78GiB   0.07%     704kB / 295kB     1.33MB / 4.1kB    12
2befa5505fc0   sample_docker_carla_project_ros_talker_1     0.12%     31.25MiB / 30.78GiB   0.10%     709kB / 320kB     15.2MB / 4.1kB    12
```
