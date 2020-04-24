import sys
sys.path.append('/home/jonas/Documents/Repos/3D_Vision_AR_RobotVis/AR_Manager/src')
sys.path.append('/home/jonas/Documents/Repos/3D_Vision_AR_RobotVis/AR_Manager/')

import logging
import yaml

from comp import RosComponent, UnityComponent, Instance

import copy

import docker

"""
This will be implemented based on flask and REST-API
"""

class Server():
	"""
	implements all functionality
	"""
	def __init__(	self, \
								cfg_ros_comp = 'src/cfg/cfg_ros_comp.yml', \
								cfg_unity_comp = 'src/cfg/cfg_unity_comp.yml'):
		"""
		cfg_ros_comp = stores all information about available ros comp
		  is read in when initalized
		cfg_unity_comp = stores all information about available unity comp
		  is read in when initalized
		every client is able to start or stop components

		Idee of components: 
		YML-files -> ComponentList
		RosComponent UR3
		RosComponent UR5

		call create_component(UR3)
			if UR3 in list:
				if CompInstanceCounter < UR3.cfg:
					copy UR3 to instances list
					instance UR3.start()
					increase ComponentInstance counter

		Extension later: Allow groups in config (set of mutiple instances) (nice to have)
		In most cases this will be done by roslaunch directly
		"""
		self._docker_client = docker.from_env()

		self._avail_comps = cfg_to_comps( cfg_ros_comp, self._docker_client)
		self._avail_comps += cfg_to_comps( cfg_unity_comp, self._docker_client)
		
		#containes all running components
		#components are always copied and then they are instances
		#instances might be ros or unity comp
		self._instances = [] 
		self._instance_counter = 0 



	def __str__(self):
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

	def server_close(self):

		logging.info('Close Server')
		self.stop_instances()
		
	def stop_instances(self):

		for inst in self._instances:
			inst.stop()
		

	def start(self, comp_name):

		for comp in self._avail_comps:
			if comp.name == comp_name:

				self._instances.append( Instance(comp=comp,inst_id=self._instance_counter) )
				self._instance_counter += 1
				return self._instances[-1].getData()

		logging.error('Cant find component')
		return -1

	def remove_instance(self, inst_id):  		
		for inst in self._instances:
			if inst.id() == inst_id:
				inst.remove()
				self._instances.remove(inst)
				return {'suc': True}	
		
		return -1
		logging.error('Cant find instance id')


	def spin(self):
		"""
		updates all component states
		spin must be called cyclic
		removes stoped instances
		"""
		for inst in self._instances:
			running = inst.update()
			if not running: 
				self.remove_instance(inst.id)
		
	def get_instance(self, inst_id):
		for inst in self._instances:
			
			if inst.id == inst_id:
				return inst.getData()
		print("id not found")
		return -1

	def get_instances(self):
		ls = []
		for inst in self._instances:
			ls.append( inst.getData() )

		if len(ls) == 0:	
			return {'suc': False}
		return ls
		
	def add_comps(self,cfg,store=False):
		to_remove =[]
		for i in range(0,len(cfg)): 
			if self.get_avail_comp(cfg[i]['pretty_name']) != -1:
				logging.warning('Component with name {} already exists'.format(cfg[i]['pretty_name']))
				to_remove.append(cfg[i])
		for i in to_remove:
			cfg.remove(i)

		self._avail_comps += cfg_to_comps( cfg, self._docker_client )

	def get_avail_comps(self):

		ls = []
		for comp in self._avail_comps:
			ls.append( comp.getData() )

		if len(ls) == 0:	
			return -1
		return ls

	def get_avail_comp(self, name):

		for comp in self._avail_comps:
			if comp.name == name:
				
				return comp.getData()
		return -1

	def remove_avail_comp(self, name):

		for comp in self._avail_comps:
			if comp.name == name:
				self._avail_comps.remove(comp)
				return comp.getData()
		return -1

def cfg_to_comps(cfg_file, docker_client):

	"""
	reads in yml-file: 
	creates component objects based on this file

	ToDo: Check if cfg file is valid
	"""
	#open the template
	print(type(cfg_file))

	if isinstance(cfg_file, str):
		with open(cfg_file) as f:
			data = yaml.load(f, Loader=yaml.FullLoader)['components']
		print('data[components]' , data)
	elif isinstance(cfg_file, list):
		data = cfg_file
		print("GOT CFG AS LIST")
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



