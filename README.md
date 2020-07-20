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
At first have a look at [client implementation](https://github.com/EricVoll/ARbotics/blob/ros_docker_dev/AR_Manager/src/client/python_rest_demo.py)

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



```


** dns (list) ** – Set custom DNS servers.
dns_opt (list) – Additional options to be added to the container’s resolv.conf file.
dns_search (list) – DNS search domains.
domainname (str or list) – Set custom DNS search domains.
entrypoint (str or list) – The entrypoint for the container.
environment (dict or list) – Environment variables to set inside the container, as a dictionary or a list of strings in the format ["SOMEVARIABLE=xxx"].
extra_hosts (dict) – Additional hostnames to resolve inside the container, as a mapping of hostname to IP address.
group_add (list) – List of additional group names and/or IDs that the container process will run as.
healthcheck (dict) – Specify a test to perform to check that the container is healthy.
hostname (str) – Optional hostname for the container.
init (bool) – Run an init inside the container that forwards signals and reaps processes
init_path (str) – Path to the docker-init binary
ipc_mode (str) – Set the IPC mode for the container.
isolation (str) – Isolation technology to use. Default: None.
kernel_memory (int or str) – Kernel memory limit
labels (dict or list) – A dictionary of name-value labels (e.g. {"label1": "value1", "label2": "value2"}) or a list of names of labels to set with empty values (e.g. ["label1", "label2"])
links (dict) – Mapping of links using the {'container': 'alias'} format. The alias is optional. Containers declared in this dict will be linked to the new container using the provided alias. Default: None.
log_config (LogConfig) – Logging configuration.
lxc_conf (dict) – LXC config.
mac_address (str) – MAC address to assign to the container.
mem_limit (int or str) – Memory limit. Accepts float values (which represent the memory limit of the created container in bytes) or a string with a units identification char (100000b, 1000k, 128m, 1g). If a string is specified without a units character, bytes are assumed as an intended unit.
mem_reservation (int or str) – Memory soft limit.
mem_swappiness (int) – Tune a container’s memory swappiness behavior. Accepts number between 0 and 100.
memswap_limit (str or int) – Maximum amount of memory + swap a container is allowed to consume.
mounts (list) – Specification for mounts to be added to the container. More powerful alternative to volumes. Each item in the list is expected to be a docker.types.Mount object.
name (str) – The name for this container.
nano_cpus (int) – CPU quota in units of 1e-9 CPUs.
network (str) – Name of the network this container will be connected to at creation time. You can connect to additional networks using Network.connect(). Incompatible with network_mode.
network_disabled (bool) – Disable networking.
network_mode (str) –
One of:

bridge Create a new network stack for the container on on the bridge network.
none No networking for this container.
container:<name|id> Reuse another container’s network stack.
host Use the host network stack.
Incompatible with network.

oom_kill_disable (bool) – Whether to disable OOM killer.
oom_score_adj (int) – An integer value containing the score given to the container in order to tune OOM killer preferences.
pid_mode (str) – If set to host, use the host PID namespace inside the container.
pids_limit (int) – Tune a container’s pids limit. Set -1 for unlimited.
platform (str) – Platform in the format os[/arch[/variant]]. Only used if the method needs to pull the requested image.
ports (dict) –
Ports to bind inside the container.

The keys of the dictionary are the ports to bind inside the container, either as an integer or a string in the form port/protocol, where the protocol is either tcp, udp, or sctp.

The values of the dictionary are the corresponding ports to open on the host, which can be either:

The port number, as an integer. For example, {'2222/tcp': 3333} will expose port 2222 inside the container as port 3333 on the host.
None, to assign a random host port. For example, {'2222/tcp': None}.
A tuple of (address, port) if you want to specify the host interface. For example, {'1111/tcp': ('127.0.0.1', 1111)}.
A list of integers, if you want to bind multiple host ports to a single container port. For example, {'1111/tcp': [1234, 4567]}.
privileged (bool) – Give extended privileges to this container.
publish_all_ports (bool) – Publish all ports to the host.
read_only (bool) – Mount the container’s root filesystem as read only.
remove (bool) – Remove the container when it has finished running. Default: False.
restart_policy (dict) –
Restart the container when it exits. Configured as a dictionary with keys:

Name One of on-failure, or always.
MaximumRetryCount Number of times to restart the container on failure.
For example: {"Name": "on-failure", "MaximumRetryCount": 5}

runtime (str) – Runtime to use with this container.
security_opt (list) – A list of string values to customize labels for MLS systems, such as SELinux.
shm_size (str or int) – Size of /dev/shm (e.g. 1G).
stdin_open (bool) – Keep STDIN open even if not attached.
stdout (bool) – Return logs from STDOUT when detach=False. Default: True.
stderr (bool) – Return logs from STDERR when detach=False. Default: False.
stop_signal (str) – The stop signal to use to stop the container (e.g. SIGINT).
storage_opt (dict) – Storage driver options per container as a key-value mapping.
stream (bool) – If true and detach is false, return a log generator instead of a string. Ignored if detach is true. Default: False.
sysctls (dict) – Kernel parameters to set in the container.
tmpfs (dict) –
Temporary filesystems to mount, as a dictionary mapping a path inside the container to options for that path.

For example:

{
    '/mnt/vol2': '',
    '/mnt/vol1': 'size=3G,uid=1000'
}
tty (bool) – Allocate a pseudo-TTY.
ulimits (list) – Ulimits to set inside the container, as a list of docker.types.Ulimit instances.
use_config_proxy (bool) – If True, and if the docker client configuration file (~/.docker/config.json by default) contains a proxy configuration, the corresponding environment variables will be set in the container being built.
user (str or int) – Username or UID to run commands as inside the container.
userns_mode (str) – Sets the user namespace mode for the container when user namespace remapping option is enabled. Supported values are: host
uts_mode (str) – Sets the UTS namespace mode for the container. Supported values are: host
version (str) – The version of the API to use. Set to auto to automatically detect the server’s version. Default: 1.35
volume_driver (str) – The name of a volume driver/plugin.
volumes (dict or list) –
A dictionary to configure volumes mounted inside the container. The key is either the host path or a volume name, and the value is a dictionary with the keys:

bind The path to mount the volume inside the container
mode Either rw to mount the volume read/write, or ro to mount it read-only.
For example:

{'/home/user1/': {'bind': '/mnt/vol2', 'mode': 'rw'},
 '/var/www': {'bind': '/mnt/vol1', 'mode': 'ro'}}
volumes_from (list) – List of container names or IDs to get volumes from.
working_dir (str) – Path to the working directory.
Returns:	
The container logs, either STDOUT, STDERR, or both, depending on the value of the stdout and stderr arguments.

STDOUT and STDERR may be read only if either json-file or journald logging driver used. Thus, if you are using none of these drivers, a None object is returned instead. See the Engine API documentation for full details.

If detach is True, a Container object is returned instead.

Raises:	
docker.errors.ContainerError – If the container exits with a non-zero exit code and detach is False.
docker.errors.ImageNotFound – If the specified image does not exist.
docker.errors.APIError – If the server returns an error.