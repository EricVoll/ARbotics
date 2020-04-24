import logging
import time
from abc import ABC, abstractmethod

import datetime 
from dataclasses import dataclass
import docker 
from dataclasses import dataclass
from dataclasses_json import dataclass_json

@dataclass_json
@dataclass
class AvailCompInfo:
	PRETTY_NAME: str
	max_instances: int
	instances: int
	comp_type: str

@dataclass_json
@dataclass
class InstInfo:
	inst_id : int
	start_time: float
	stop_time: float
	active: bool
	running: bool
	killed: bool
	stop: bool
	docker_id: str

@dataclass_json
@dataclass
class RosCompInfo:
	urdf_content: str
	urdf_tooltip: str
	docker_cmd: str
	docker_image: str

class Instance():
	def __init__(self, comp, inst_id):
		"""
		an instances manages a container.

		if active instance is doing stuff 
		if running container is running

		if killed container is closed 
		if stop (instances tries to stop container)

		"""
		self._comp = comp #base component
		self._ii = InstInfo(
			inst_id = inst_id,
			start_time= time.time(),
			stop_time= None,
			active = True, 
			running = False,
			killed = False,
			stop = False,
			docker_id ='TBD' 	)
		print("instance launched"+ self._comp.name, self.getData() )
		self._container = self._comp.start()
	
	def __str__(self):
			return 'Comp: %15s,   ID: %3s,   Started: %10s ,   UpTime: %10s'%(\
				self._comp.name, self._ii.inst_id, datetime.datetime.fromtimestamp(self._ii.start_time) , datetime.timedelta(milliseconds=self.get_uptime()) )

	def get_uptime(self):
		"""
		returns unique id which is set when instanciated
		"""
		if self._ii.active:
			return time.time() - self._ii.start_time
		else:
			return 0

	@property
	def id(self):
		return self._ii.inst_id

	@property
	def name(self):
		"""
		returns unique id which is set when instanciated
		"""
		return self._comp.name

	def remove(self):
		self._comp.instance_closed()

	def stop(self):
		self._container.stop()
		self._container.remove()
	
	def getData(self):
		#.to_dict()
		d = {'inst': self._ii.to_dict(), 'comp' : self._comp.getData()['comp']  }
		return d

	def update(self):
		if self._container.status == 'exited':
			return False

		return True
		

class Component(ABC):
	"""
	a component defines how to actually interact with a container
	"""
	def __init__ (self, cfg, docker_client):
		self._aci = AvailCompInfo(
			comp_type= cfg['comp_type'], 
			PRETTY_NAME = cfg['pretty_name'],
			max_instances = cfg['max_instances'],
			instances = 0
		)
		self.docker_client = docker_client

	def __str__(self):
		return 'Comp: %15s, Instances: %3s'%(self._aci.PRETTY_NAME, self._aci.instances)

	def instance_closed(self):

		if self._aci.instances > 0: 
			self._aci.instances -= 1
		else:
			logging.error('Confirmed closed instance but no instances is running')

	def add_instance(self):
		self._aci.instances += 1

	@property
	def instances(self):
		return self._aci.instances
	
	def getData(self):
		d = {'comp': self._aci.to_dict() }
		return d
	@property
	def name(self):
		"""
		returns unique id which is set when instanciated
		"""
		return self._aci.PRETTY_NAME

	@property
	def max_instances(self):
		return self._aci.max_instances

	@property
	def available(self):
		return self._aci.instances <= self._aci.max_instances


	@abstractmethod 
	def start(self):
		self.add_instance()
		print("STARTED: ", self._aci.PRETTY_NAME)
		#goes into container and launches/runs
	
	@abstractmethod 
	def stop(self):
		print("STOP: ", self._aci.PRETTY_NAME)

		#goes into container and stops
	
	@abstractmethod 
	def update(self):
		pass



class RosComponent(Component):
		
	def __init__ (self, cfg, docker_client):
		"""
		ToDo: Check if cfg valid
		"""
		super(RosComponent, self).__init__(cfg, docker_client)
		print(cfg)
		self._rci = RosCompInfo(
			urdf_content = cfg['urdf']['content'],
			urdf_tooltip = cfg['urdf']['tooltip'],
			docker_cmd = cfg['docker']['cmd'],
			docker_image = cfg['docker']['image'])


	def start(self):

		super(RosComponent, self).start()
		cmd = '''bash -c '%s' '''%self._rci.docker_cmd
		
		container = self.docker_client.containers.run(self._rci.docker_image, 
			detach=True, 
			command = cmd,
			network_mode='host' )
		print('executed docker run got container:', container)
		return container 
		#goes into container and launches/runs

	def stop(self):

		super(RosComponent, self).stop()
		#goes into container and stops

	def update(self):

		super(RosComponent, self).update()


class UnityComponent(Component):

	def __init__ (self, cfg, docker_client):
		"""
		ToDo: Check if cfg valid
		"""
		super(UnityComponent, self).__init__(cfg, docker_client)

	def start(self): 
		super(UnityComponent, self).start()
		#goes into container and launches/runs

	def stop(self):
		super(UnityComponent, self).stop()
		#goes into container and stops

	def update(self):
		super(UnityComponent, self).update()



#class AvailCompInfo():
if __name__ == '__main__':
	#inst = AvailCompInfo('s1',2,'s3','s5','s5',1)

	# inst.docker_info = 'assda'
	# print(inst.docker_info)
	# print(inst)


	client = docker.from_env()
	
	containers = client.containers.list()
	print(containers)
	# cmd = '/bin/bash  ' +\
	# 		' -c "echo "HELLO" && source /opt/ros/melodic/setup.bash &&' +\
	# 		' source /home/catkin_ws/devel/setup.bash &&' +\
	# 		' source /home/ros-sharp_ws/devel/setup.bash &&' +\
	# 		' roslaunch ur_rossharp ur5_rossharp.launch" '
	# print(cmd)

	# command="/bin/bash",
	cmd = '''bash -c 'roscore' ''' 

	#cmd = '''bash -c 'roslaunch' '''

	cmd = '''bash -c 'source /opt/ros/melodic/setup.bash &&  source /home/ros-sharp_ws/devel/setup.bash && source /home/catkin_ws/devel/setup.bash && roslaunch ur_rossharp ur5_rossharp.launch' ''' 
	
	container = client.containers.run("ros1_ur_rossharp:1.0", detach=False, command = cmd, network_mode='host' )
	
	print(container)

	# cmd = '/bin/bash -c "echo hel123lo stdout ; echo hello stderr >&2"'
	# res = container.exec_run(cmd, stream=False, demux=False)
	# print(res.output)




	