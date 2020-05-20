import sys
sys.path.append('/home/jonas/Documents/Repos/3D_Vision_AR_RobotVis/AR_Manager/src')
sys.path.append('/home/jonas/Documents/Repos/3D_Vision_AR_RobotVis/AR_Manager/')

import logging
import yaml
import copy
import docker

from comp import RosComponent, UnityComponent, Instance
from ros_nodes import ARServerStatePublisher



class Server():
	"""
	Returns an Server object that handles ROS and Unity Components. 
	Each components runs in an individual Docker Container. 
	The Server can run and stop Docker Container.
	"""
	def __init__(	self,
								cfg_ros_comp = 'src/cfg/cfg_ros_comp.yml',
								cfg_unity_comp = 'src/cfg/cfg_unity_comp.yml'):
		"""Launchs the ArServerStatePublisher ROS Node and converts both cfg into Component objects.

		Parameters
		----------
		cfg_ros_comp : str, optional
				path to the cfg file containing all ROS-Components, by default 'src/cfg/cfg_ros_comp.yml'
		cfg_unity_comp : str, optional
				path to the cfg file containing all Unity-Components, by default 'src/cfg/cfg_unity_comp.yml'
		"""					

		self._docker_client = docker.DockerClient(base_url='unix:///var/run/docker.sock' )
		self._aassp = ARServerStatePublisher()

		# self._avail_comps is a list containing all available components.
		self._avail_comps = cfg_to_comps( cfg_ros_comp, self._docker_client)
		self._avail_comps += cfg_to_comps( cfg_unity_comp, self._docker_client)
		
		# self._instances is a list containing all running components.
		# the list is automatically updated if a component closes. 
		# an instance is a copy of and available componet.
		self._instances = [] 
		self._instance_counter = 0 


	def __str__(self):
		"""String representation of the Server listing all available components and running

		Returns
		-------
		string
				all running components and instances.
		"""	
		string = "Server Summary:\n"
		
		string += "="*45+' Instances '+'='*45 +'\n'
		for inst in self._instances: 
			string += ' {}\n'.format(inst)
		string += '-'*101+'\n'+'\n'

		string += "="*45+' Comp '+'='*45 +'\n'
		for comp in self._avail_comps: 
			string += ' {}\n'.format(comp)
		string += '-'*101+'\n'+'\n'

		return string

	def ros_publish(self):
		"""Publishes server state via the ARServerStatePublisher ros node.
		"""	
		data = self.get_instances()
		self._aassp.publish(data= { 'data': data } )

	def server_close(self):
		"""Sends stop signal to all running instances.
		"""		
		logging.info('Close Server')
		self.stop_instances()
		
	def stop_instances(self):
		"""Sends stop signal to all running instances.
		"""
		for inst in self._instances:
			inst.stop()
		
	def stop_instance(self, inst_id):
		"""Sends stop signal to an individual running instance.

		Parameters
		----------
		inst_id : int
				ID of the instance that should be stopped.

		Returns
		-------
		dict
				containing all running instances after stopping the specified one
		Raises
		------
		ValueError
				Invalid inst_id. No instance in self._instances with the given ID was found
		"""				
		for inst in self._instances:
			if inst.id == inst_id:
				inst.stop()
				return self.get_instances()
		
		raise ValueError("Invalid comp_name. No component in self._avail_comps with this com_name found")
		
	def start(self, comp_name):
		"""Creats an instance from the specified available component

		Parameters
		----------
		comp_name : string
				Name of the component from which a instance should be started

		Returns
		-------
		dict
				contaning the started instances data

		Raises
		------
		ValueError
				Invalid comp_name. No component in self._avail_comps with this com_name found
		"""		

		for comp in self._avail_comps:
			if comp.name == comp_name and comp.available:

				self._instances.append( Instance(comp=comp,inst_id=self._instance_counter) )
				self._instance_counter += 1
				return self._instances[-1].get_data()

		raise ValueError("Invalid comp_name. No component in self._avail_comps with this com_name found")

	def remove_instance(self, inst_id):
		"""Removes an instance from self._instances

		Parameters
		----------
		inst_id : int
				Id of instance that should be removed

		Returns
		-------
		dict
				True if remove operation was successfull

		Raises
		------
		ValueError
				Invalid inst_id. No instance in self._instances with this ID found
		"""		
		for inst in self._instances:
			if inst.id == inst_id:
				inst.remove()
				self._instances.remove(inst)
				return {'suc': True}	
		
		raise ValueError("Invalid inst_id. No instance in self._instances with the given ID was found")

	def spin(self):
		"""This method should be called periodically.
		Checks container state of each running instance. 
		Removes the instance from list if docker-container stopped.
		"""		
		for inst in self._instances:
			running = inst.update()
			if not running: 
				try:
					self.remove_instance(inst.id)
				except ValueError:
					pass

	def get_instance(self, inst_id):
		"""Access the data of a specific instance

		Parameters
		----------
		inst_id : int
				ID of target instance

		Returns
		-------
		dict
				contains all information of instance
		
		Raises
		------
		ValueError
				Invalid inst_id. No instance in self._instances with the given ID was found
		"""	

		for inst in self._instances:
			
			if inst.id == inst_id:
				return inst.get_data()
		raise ValueError("Invalid inst_id. No instance in self._instances with the given ID was found")

	def get_instances(self):
		"""Access data of all instances

		Returns
		-------
		list
				contains data of all instances
		"""	
		ls = []
		for inst in self._instances:
			
			try:
				ls.append( inst.get_data())
			except:
				pass

		return ls

	def update_instance_urdf(self, inst_id, data):
		"""Updates the dynamic urdf of instance

		Parameters
		----------
		inst_id : int
				ID of target instance
		data : string
				new urdf content

		Returns
		-------
		dict
				data of modified instance

		Raises
		------
		ValueError
				Invalid inst_id. No instance in self._instances with the given ID was found
		"""	
		for inst in self._instances:
			
			if inst.id == inst_id:
				inst.update_urdf_dyn(data)
				return inst.get_data()

		logging.warning("id not found")
		raise ValueError("Invalid inst_id. No instance in self._instances with the given ID was found")
		
	def add_comps(self,cfg,store=False):
		"""Adding a new component to the available components of the Server

		Parameters
		----------
		cfg : dict
				cfg containing the information about the new component. See src/cfg yaml for reference
		store : bool, optional
				Flag if the given cgf should be written to the cfg.yaml, by default False
		"""		
		to_remove =[]
		for i in range(0,len(cfg)): 
			try:
				res = self.get_avail_comp(cfg[i]['pretty_name']) 
				logging.warning('Component with name {} already exists'.format(cfg[i]['pretty_name']))
				to_remove.append(cfg[i])
			except:
				pass
		for i in to_remove:
			cfg.remove(i)

		self._avail_comps += cfg_to_comps( cfg, self._docker_client )

	def get_avail_comps(self):
		"""Get data of all available components

		Returns
		-------
		dict
				contains list all available components data
		"""	
		ls = []
		for comp in self._avail_comps:
			ls.append( comp.get_data()['comp'] )

		if len(ls) == 0:	
			return -1
		return {'components': ls}

	def get_avail_comp(self, name):
		"""Get data of given component

		Parameters
		----------
		name : str
				target component name

		Returns
		-------
		dict
				data of target component

		Raises
		------
		ValueError
				Invalid name. No component in self._avail_comps with the given name was found
		"""	

		for comp in self._avail_comps:

			if comp.name == name:
				
				return comp.get_data()

		raise ValueError("Invalid name. No component in self._avail_comps with the given name was found")

	def remove_avail_comp(self, name):
		"""removes target component from self._avail_comps 

		Parameters
		----------
		name : string
				target component name

		Returns
		-------
		dict
				data of removed component
		
		Raises
		------
		ValueError
				Invalid name. No component in self._avail_comps with the given name was found
		"""	

		for comp in self._avail_comps:
			if comp.name == name:
				self._avail_comps.remove(comp)
				return comp.get_data()
		raise ValueError("Invalid name. No component in self._avail_comps with the given name was found")

def cfg_to_comps(cfg_file, docker_client):
	"""Helper function to create a list of ROS and Unity components from a given cfg file

	Parameters
	----------
	cfg_file : string
			path to cfg file specifying the components that should be created
	docker_client : DockerClient
			DockerClient of host system that has all containers build and ready to start them.
			A DockerClient reference will be passed to each component so it can start and stop its container

	Returns
	-------
	list
			Contains RosComponents and UnityComponents.
	"""

	if isinstance(cfg_file, str):
		with open(cfg_file) as f:
			data = yaml.load(f, Loader=yaml.FullLoader)['components']
	elif isinstance(cfg_file, list):
		data = cfg_file
	else:
		logging.error('invalid cfg type')
	
	comps = []
	for single_cfg in data:
		if single_cfg['comp_type'] == 'ros':
			comps.append( RosComponent(single_cfg, docker_client) )
		elif single_cfg['comp_type'] == 'unity':
			comps.append( UnityComponent(single_cfg, docker_client) )
		else:
			logging.warning('Comp type ({}) not defined'.format(single_cfg['comp_type']))
	return comps



