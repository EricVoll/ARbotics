import sys
sys.path.append('/home/jonas/Documents/Repos/3D_Vision_AR_RobotVis/AR_Manager/src')
sys.path.append('/home/jonas/Documents/Repos/3D_Vision_AR_RobotVis/AR_Manager/')

import logging
import yaml
import copy
import docker

from comp import RosComponent, UnityComponent, Instance
from ros_nodes import ARServerStatePublisher


"""
This will be implemented based on flask and REST-API

joint_states
name: [elbow_joint, shoulder_lift_joint, shoulder_pan_joint, wrist_1_joint, wrist_2_joint,
  wrist_3_joint]
"""

class Server():
	"""
	implements all functionality
	"""
	def __init__(	self, \
								cfg_ros_comp = 'src/cfg/cfg_ros_comp.yml', \
								cfg_unity_comp = 'src/cfg/cfg_unity_comp.yml'):
		"""
		additional error handling needed for DockerClient
		"""

		self._docker_client = docker.DockerClient(base_url='unix:///var/run/docker.sock' )
		self._aassp = ARServerStatePublisher()
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

	def ros_publish(self):
		data = self.get_instances()
		self._aassp.publish(data= { 'data': data } )

	def server_close(self):

		logging.info('Close Server')
		self.stop_instances()
		
	def stop_instances(self):

		for inst in self._instances:
			inst.stop()
		
	def stop_instance(self, inst_id):

		for inst in self._instances:
			if inst.id == inst_id:
				inst.stop()
				return self.get_instances()
		logging.error('Instance {} not found to stop'.format(inst_id))
		return -1

	def start(self, comp_name):

		for comp in self._avail_comps:
			if comp.name == comp_name and comp.available:

				self._instances.append( Instance(comp=comp,inst_id=self._instance_counter) )
				self._instance_counter += 1
				return self._instances[-1].get_data()

		logging.error('Component {} not found to start'.format(comp_name))

		return -1

	def remove_instance(self, inst_id):  		
		print("remove instance from list")
		for inst in self._instances:
			if inst.id == inst_id:
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
				return inst.get_data()
		print("id not found")
		return -1

	def get_instances(self):
		ls = []
		for inst in self._instances:
			ls.append( inst.get_data())

		if len(ls) == 0:	
			return []
		return ls

	def update_instance_urdf(self, inst_id, data):
		for inst in self._instances:
			
			if inst.id == inst_id:
				inst.update_urdf_dyn(data)
				return inst.get_data()
		logging.warning("id not found")
		return -1
		
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
			ls.append( comp.get_data()['comp'] )

		if len(ls) == 0:	
			return -1
		return {'components': ls}

	def get_avail_comp(self, name):

		for comp in self._avail_comps:

			if comp.name == name:
				
				return comp.get_data()
		return -1

	def remove_avail_comp(self, name):

		for comp in self._avail_comps:
			if comp.name == name:
				self._avail_comps.remove(comp)
				return comp.get_data()
		return -1

def cfg_to_comps(cfg_file, docker_client):

	"""
	reads in yml-file: 
	creates component objects based on this file

	ToDo: Check if cfg file is valid
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



