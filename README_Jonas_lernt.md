
## Linux ROS Installation:
### Install ros melodic 
- 100% follow tutorial [offical instructions](http://wiki.ros.org/melodic/Installation/Ubuntu)
- Ubuntu 18.04 

### Install anaconda3 
- Choose your home folder as the default path  
- conda install defusedxml
- Add here deeplearning part
- Create conda ros enviorment (see the provided YML file for the 
enviroment ) [full guide for env](https://docs.conda.io/projects/conda/en/latest/user-guide/tasks/manage-environments.html)
- Activate the ros env by default, run:  
```
echo "conda activate ros " >> `~/.bashrc`   
echo "source /opt/ros/melodic/setup.bash" >> `~/.bashrc`
```  
### Set up catkin_workspace
setting up a workspace using python 3.7.4: [Based on offical tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
(will this results in unnecassary trouble?)
- Check your python version (3.7.4 | path to anaconda3 folder)
``` 
python -V
which python
``` 

- Create folder structure 
```
mkdir catkin_ws
cd catkin_ws
mkdir src
```
- Create the workspace with the correct python path
[See](https://medium.com/@beta_b0t/how-to-setup-ros-with-python-3-44a69ca36674)
```
catkin_make --cmake-args \
            -DCMAKE_BUILD_TYPE=Release \
            -DPYTHON_EXECUTABLE=~/anaconda3/envs/ros/bin/python \
            -DPYTHON_INCLUDE_DIR=~/anaconda3/envs/ros/include/3.7m \
            -DPYTHON_LIBRARY=~/anaconda3/envs/ros/lib/libpython3.7m.so


catkin_make -DPYTHON_EXECUTABLE=~/anaconda3/envs/ros/bin/python -DPYTHON_INCLUDE_DIR=~/anaconda3/envs/ros/include/3.7m -DPYTHON_LIBRARY=~/anaconda3/envs/ros/lib/libpython3.7m.so
```
i installed the ros thing from the medium page 
after this i could successfully run the ur package installation 
not exactly sure why but now building the workspace with catkin_make_isolated works perfectly

- Install UR package:

``` 
cd ~/catkin_ws/src
git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git 
```
```
catkin_make
#activate the workspace
source $HOME/catkin_ws/devel/setup.bash
```
```
catkin_make_isolated #solved the problem with the universal robot
```

Launch each of the commands in a seperat terminal:
```
roslaunch ur_gazebo ur5.launch
```
```
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true
```
```
roslaunch ur5_moveit_config moveit_rviz.launch config:=true
```

# Gazebo
Everything is worth mentioning from te tutorials
```
gazebo worlds/pioneer2dx.world
```
Worlds are located at:
```
ls /usr/share/gazebo-7/worlds
```

gazebo comman starts a `gzserver`(core of physics and sensor data gen) and a `gzclient`(desktop visu)
- word files are in SDF (Simulation Description Format) .world
- model files (can be included into a world files) 

The communication library currently uses the open source Google Protobuf for the message serialization and boost::ASIO for the transport mechanis
Gazebo Master same functions as ROS MASTER 

# ROS Python3 dep
sudo apt update
sudo apt install python3-catkin-pkg-modules python3-rospkg-modules python3-empy


# Install catkin build with pip3
https://catkin-tools.readthedocs.io/en/latest/installing.html

## Switched to useing Python2.7 

- Just install melodic as previously
- If you install packages make sure all dependencies are fullfilled 
- Easiest way to do this is use rosdep. Install it with apt and init it. 
- Installs all dependencies needed for your catkin_ws:
```
rosdep install --from-paths src --ignore-src -r -y
```

http://wiki.ros.org/rosdep


Markdown VS-Code:

Shortcuts	Functionality
cmd-k v	Open preview to the Side
ctrl-shift-v	Open preview
ctrl-shift-s	Sync preview / Sync source
shift-enter	Run Code Chunk
ctrl-shift-enter	Run all Code Chunks
cmd-= or cmd-shift-=	Preview zoom in
cmd-- or cmd-shift-_	Preview zoom out
cmd-0	Preview reset zoom
esc	Toggle sidebar TOC

## Docker
Full guide to useing docker
[Installation Tutorial](https://docs.docker.com/install/linux/docker-ce/ubuntu)

### Docker Image
(to deploy applications)
- can contain mutiple containers 

|Commands             |                          |
|---------------------|--------------------------|
|`docker run`           |                             
|`docker ps -a`         |lists all containers including stopped ones
|`docker ps`            |list container
|`docker stop ID/Name`  |
|`docker rm ID/Name `   |
|                       |
|`docker images`    |see all images and sizes 
|`docker rmi ID/Name`   |removes image. Before stop all running and dependent containers
|`docker pull`  |   |
containers are not mend to host a operating system
once the task is complete the container exits. 
therefore the container of ubuntu stops immediately.

Executing a container and run a command

`docker run ubuntu sleep 1000    `



`docker exec` something

`docker run -d name `# runs detached in background mode
`docker run -i `
interactive mode able to IO data
`docker run -it` attach to terminal in interactive mode of the container

`docker attach ID/name`

### Port Mapping 
docker run tells the port it is running. This is an internal IP and can be only accessed by the docker host
Two options access 
- Internal IP 
- Map port of Docker Container to Free Port of Docker host -> Therefore container can be accessed from outside

```docker run -p 80:5000 something ```
This maps port 80 of docker host to port 5000 of docker container
User can access with: 192.168.1.5:50  

```docker run -d --name=ros2 ```

### Volume mapping 
docker container has own isolated file system
when remove a container. It and all its data is gone. Therfore map directorie outside the container from the local host to the container
```docker run -v /opt/datadir:/car/lib/mysql something```

Notation always host:container
 

```docker inspect ```
gives you additional infos
```docker logs ```

### Working with enviorment variables

```python: 
import os 
os.environ.get('APP_COLOR')
```
Setting Enviorment Variable when running container:
```
docker run -e APP_COLOR=blue
```
With the inspect you can see Config Env the set environment variables 

Create your own Image:  
Containerizing.   
- What do we want to Container 
- OS - Ubuntu 
- Update 

Docker files  

```docker history```
shows history of an image. Gives size of individual installed dependencies ...  

```docker image build -t nice_image_name:1.0 .``` lets you build a Dockerfile.  
Build process works in layer stages   
Rebuilding image is faster. Since only layers above updated layers must be rebuild!  
sudo docker image build -t bulletinboard:1.0 .

#### Creating Docker Images

https://kapeli.com/cheat_sheets/Dockerfile.docset/Contents/Resources/Documents/index

`FROM`
`RUN`
`COPY` host/dir container/dir                  <- this will be during build
`ADD` hhtps://download_this container/dir      <- this will run when executing
`ENV`
`Entrypoint` do stuff here that is done when the container is started 
`Workdir`:
`CMD` ["bash"]



#### Bridge Network
```docker run ubuntu``
all containers are attached to this netwokr by default 
172.17.0.1 = docker0 
Map ip adresss of an individual container to an host port
all container cann communication internally over docker0 
Keeps network isolation

```docker run Ubuntu --network=none```
complete isolated from host 


```docker run Ubuntu --network=host```
breaks network isolation 
container run now on one port 
Web 5000 therfore you cant run another Web 5000 in docker. 



```docker network ls``` lists all running networks
Embedded DNS server: Allows to connect container simply by names instead of IP-Addresses
DNS Server runs at 127.0.0.11


### Docker Storage 
File system
/var/lib/docker
    aufs
    containers
    image 
    volumes

#### Copy-on-write 
so basically we have a Image Layer which is constant and defined during the build. If you want to change anything. First it is copied to the Container Layer where it can be modified. Image will remain the same. 
When we get rid of the container everythin in the container will be deleted. 

```docker volume create data_volume```
creates a volume on Docker Host at /var/lib/docker/volumes 

can now pars this to a docker container in the run statement by adding
```-v data_volume:/var/lib/mysql``` to the ```docker run``` command

if volume has not be created this will be done atocamtically
this is called volume mounting. If we have a external storage on the host then run a container with the complete path of the host 
This is called bind mount 
mounts a directory on th docker host

```
docker run \
-- mount type=bind,source=/data/mysql, target=/var/lib/myswl \
mysql
```

Execute a second shell for container
```
sudo docker exec -it container_name bash 

```
#### docker-compose.yml 
same as in ansible
run containers on a single docker host


```
docker-compose up -d
```
https://docs.docker.com/compose/compose-file/



### docker run --link 

## Install Gazebo sepratly on local linux distribution. 
create communication to ROS2 container via network 


# ROS Commands
DDS Data Distribution Servive (Peer-to-peer) -> no roscore
```ros2 pkg list``` get all packages  
```ros2 run <package> <node>```  
```ros2 pkg executables``` list of packages + executables  

```ros2 topic list```
```ros2 service list ```


Create a package for Python
Go to workspace and execute in /src dir
ros2 pkg create <name>
always use template: 
delete CMake.txt

include
setup.py 
package.xml -> change only name build_type ament_python
setup.cfg 

#### Template looks like this
https://github.com/ros2/examples
```
package_name_py  
    /include  
    /package_name_py  
        __init__.py  
        module1.py  
        module2.py  
    /src  
    /resources <- is used to install data_files defined in setup.py
        package_name_py <- empty file
    package.xml  
    setup.cfg  
    setup.py  
```
**ToDo List:**
1. Delete CMake.txt
2. Modify name in setup
3. package.xml change name and build_type to ament_python instead of ament_cmake
4. setup.py package_name, entry_point={
    'console_scripts':['name_executable = package_name_py.module1:main' ] }
5. setup.cfg change paths;where to install in the lib folder


```colcon build --symlink-install``` to compile our package 
-> source our workspace
source install/local_setup.bash

ROS get envronment variables 
printenv | grep -i ROS

###Unix Shell Overview
https://swcarpentry.github.io/shell-novice/reference/

Writing Bash Scripts:
https://devhints.io/bash


### install and uninstall packages
https://industrial-training-master.readthedocs.io/en/melodic/_source/session1/Installing-Existing-Packages.html


new BASH with
```exec bash```

ROS1
To find available packages, use:
```
apt search ros-melodic
```