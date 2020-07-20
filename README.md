# ARbotics




![](project_overview.png)
## Docker Overview
We provide multiple Docker Images in ```Docker``` showing the full functionality of our framework.

### Provided Images Overview 
- **ros1_base:** Base ROS1 Image to create costume container running Ubuntu 18.04LTS and ROS1 Melodic
- **ros1_ros-sharp:** Exposes Docker Network via ROSBridge
- **ros1_ar_manager:** Responsible for managing containers.  
- **swagger_server:** REST-API Documentation. Offers example REST-Client WebInterface
- **ros1_anymal:** Anymal ROS-package
- **ros1_ur:** Universal Robot ROS-package

The **ros1_base** image can be used to create custom docker images with ROS code tailored to your specific needs of the user.
An example extension of the **ros1_base** can be found under ```ROS1/ros1_ur```
Additionally a ```Docker/docker-compose.yml``` is provided to start the complete example system and build all related containers.

### Creating Custom Docker Images
To run your own ROS code in a Docker container simply create a new Dockerfile. Simply inherit from the **ros1_base** container by including `FROM ros1_base:latest` at the top of the Dockerfile. Afterwards include all command necessary to install your ROS-Code. 

One handy feature that eases Container development is using VisualStudio Codes Docker Plugin. With this you can simply develop inside a container your code with the popular VSCode GUI. Given the isolation between your highly costmized personal developer computer and the docker container you avoid mutiple problems before arising. Additionally you don`t have to worry about incompatitle version or crashing your personal Operating System while developing new features. Here is the offical VSCode Remote container tutorial: https://code.visualstudio.com/docs/remote/containers
We highly recommend using this or other tools if you ar not absolutly familar with the command line. 

### AR-Manager Getting Started
The example docker containers are started via:
```cd Docker && sudo docker compose-up ```

If the images are not build, all images will be build. This might take several minutes.
After successfully running the command the AR-Manager is ready to accept REST requests to start and stop containers.
The rest server can be accessed on port 5000 of the local host network.
By navigating in the Webbrowser of your choice to http://localhost:5002 the Swagger-ui REST-API documentation can be accessed.

### SwaggerUI API-Documentation
Swagger-ui is an open source project to document REST-APIs. The complete configuration of the Webside is provided by a single json config at ```Docker\swagger_config\AR-Manager_swagger_cfg.json```. 
We refer the reader to the documentation Swager-ui for further instructions on how to extend this API-Documentation. 

For sending custom REST request without Swagger-UI we recommend using Postman.

#### REST-API 
The AR_Manager manges components. A component is either of comp_type: [ros, unity]. Each component represents a container that can be instanciated within the docker network offering a specific functionality (robot or sensor) or Unity GameObject for example a simulated camera, or lidar sensor. A component can be started by a REST-POST request. The AR-Manager accepts the request and creats an instance of the given component. This instance can be stopped by a REST-DELETE request. The full functionality of the REST-API can be explored via the provided Swagger-UI Documentation. 

### Configuring new Components
There are two options available to add register container configrations to the AR_Manger.
The first option is to adjust the standard configuration file provided in ```AR_Manager/src/cfg/cfg_ros_comp.yml```
This file is loaded when the AR_Manager is started and therefore contains the base functionality of the ARbotics.

Each component must include the following parameters:
```
comp_type: [ros, unity]
docker: [native docker run python interface] 
max_instances: [maximum instances allowed to run simultainously]
pretty_name: [unique pretty front end name for the component]
urdf: 
	dyn: [xml of the robot]
	stat: [xml containg custom tags to define additional information]
```
Here the docker parameters offer to include Docker run parameters to the configured container by the native docker python API. 
All arguments present in the docker run command can be passed as a argument and found here: https://docker-py.readthedocs.io/en/stable/containers.html 




![](OverviewARManagerREST.svg)

## Starting the AR-Manager:
1. Build all containers and start everything:

```
cd <PATH>/ARbotics/Docker/
docker-compose up
```

Replace <0428> with container ID 
```
docker exec 0428 bash -c "source /opt/ros/melodic/setup.bash &&  source /home/catkin_ws/devel/setup.bash && \
export ROS_MASTER_URI=http://ros1_base:11311 && \
python /home/catkin_ws/src/universal_robot/ur_rossharp/moveit_test.py"
```

```
docker exec 0428 bash -c "source /opt/ros/melodic/setup.bash &&  source /home/catkin_ws/devel/setup.bash && \
export ROS_MASTER_URI=http://ros1_base:11311 && \
python /home/catkin_ws/src/universal_robot/ur_rossharp/ur5_demo_script.py"
```


## REST-API
At first have a look at [client implementation](https://github.com/luchspeter/ARbotics/blob/ros_docker_dev/AR_Manager/src/client/python_rest_demo.py)

To test REST_API use google chrome plug in ```Advanced REST```
**http://127.17.0.1:5000**

**/AvailComps**
- **GET**: returns avail comps
- **POST** [json -> comonent_full_infos]: adds new cfg to AvailComps

**/AvailComps/<str:comp_name>**
- **GET**: returns avail comp
- **DELETE**: delets this comp -> cant be started

**/AvailInstances**
- **GET**: returns all running instances
- **POST** [json -> {comp_name:  'UR5'}]: starts instance of comp_name
- **DELETE** []: deletes all

**/Instances/<int:instance_id>**
- **GET**: returns this running instance
- **POST** [json -> {comp_name:  'UR5'}]: starts instance of comp_name
- **DELETE** []: this instance with instance_id

**/Instances/<int:inst_id>/inst/urdf_dyn**
- **POST** [json -> {data: 'updated urdf as str'}]: string argument

### Debugging Commands
```
docker stop $(docker ps)
```
```
docker run -it -v /var/run/docker.sock:/var/run/docker.sock --network=host ros1_ar_manager
```

```
docker exec ros1_ur_rossharp 


# Docker 
[Installation Tutorial](https://docs.docker.com/install/linux/docker-ce/ubuntu)
## Docker commands:
```docker build -t nice_image_name:1.0 .```
`-t` tages image with name; . navigates to where Dockerfile is located(here in same dir)

```docker run ros1_base:1.0```
`-d` for detached. It is running in the background
`-i` interactive
`-it` interactive and attached to terminal
`-network=host` run the container in non-isolated network. (same as on your host PC)
`-p 80:5000` maps port 80 of docker host to port 5000 of docker container
`-v ./shared_volume:/home/shared_volume` maps folder at ./shared_volume form host to /home/shared_volume in container

```docker ps``` let's you see all runing containers
'-a' even stoped ones

```docker stop ID/Name```  
```docker rm ID/Name```

```docker exec ID/Name```
```docker exec -it ID/Name bash ``` Executes a second shell in running container

```sudo docker system prune``` deletes all cache files 
`-a` also all images
## Structure
### Design Considerations ROS
Every ROS-node will belong to a single container. 
Three different base images are provided for ROS1, ROS2, ROS1+2
This allows to develop on a single node without risking to break anything.
Modules are clearly seperated. 
Communication within the containers is done by highly optimized Docker Container Networking Model.
Different network isolation options between the Host and Containers are available.

If you want to create or modify a new ROS-Node simply take one of the base images and modify it.

Simplest way to go is copy for example `ros2_example` folder and rename it.
(Make sure to use the `FROM` command to create new base image)  
Create and `newNode_install.sh` file in this folder.  
Build the container and run it with the volume mapped so you can modify the `newNode_install.sh` with VSCode. (Use argument `docker run -v`) Do the bugfixing and debugging until the new node works. Therefore use the `newNode_install.sh` for all installation purposes. (`chmod +x newNode_install.sh`). When finished simply add the script to the Dockerfile (see for example ros2_gazebo). If you want to develop a ROS node to the same simply map a workspace between the node and your host PC with the `docker run -V` and do the development in VSCode. 
### Dockerfile 

Install all containers:
''' 
docker build --no-cache -t ros1_base ./ros1_base/ &&\
docker build --no-cache -t ros1_ur ./ros1_ur/ &&\
docker build --no-cache -t ros1_ur_rossharp ./ros1_ur_rossharp/
'''



[Dockerfile Best Practices](https://docs.docker.com/develop/develop-images/dockerfile_best-practices/)
[Dockerfile Reference](https://docs.docker.com/engine/reference/builder/)
### Docker compose up/down
The compose command builts and runs all services/container specified in the docker-compose.yml
Similar to ansible
`docker-compose up -d`
[docker-compose Reference](https://docs.docker.com/compose/compose-file/)

#### ros1_2_bridge 
Is the bridge between ROS1 and ROS2. Problem we have to write interface nodes in ROS1 aswell as ROS2 in this docker-container

#### ros2_moveit 
We can use MoveIt2! in ROS2 here. 
Add here lauch files for different robots. (Not sure how good this is working)
Alternative we can use MoveIt! in ROS1

#### ros1_ros-sharp 
Includes the bridge to Unity.

### Useful link:
[Really nice Project ROS/Docker ETH](https://github.com/gramaziokohler/ros_docker)
[Writing Bash Scripts:](https://devhints.io/bash)
[ROS1 Training](https://industrial-training-master.readthedocs.io/en/melodic/_source/session1/Installing-Existing-Packages.html)
[Unix Shell Overview](https://swcarpentry.github.io/shell-novice/reference/)


# Literature:
Docker and ROS Tutorial: https://link.springer.com/chapter/10.1007/978-3-319-54927-9_9


