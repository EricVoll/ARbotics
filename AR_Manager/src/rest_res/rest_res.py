import sys
sys.path.append('/home/jonas/Documents/Repos/3D_Vision_AR_RobotVis/AR_Manager/src')
sys.path.append('/home/jonas/Documents/Repos/3D_Vision_AR_RobotVis/AR_Manager/')

from flask_restful import Resource, reqparse


class ResInstances(Resource):
	def __init__(self, server):

		self.s = server

	def get(self):

		data = self.s.get_instances()
		print(self.s)
		return data #returns the id of the instance

	def post(self):


		parser = reqparse.RequestParser()
		parser.add_argument('comp_name', type=str, required=True, location='json')
		args = parser.parse_args()
		res = self.s.start(args['comp_name'])
		print(self.s)
		return res

	def delete(self):
		#delete all runing comps
		
		self.s.stop_instances()

		print(self.s)
		return {'Suc':True}
	
class ResInstance(Resource):
	def __init__(self, server):

		self.s = server

	def get(self,inst_id):
		print(self.s)
		return self.s.get_instance(inst_id)
	
	def post(self,inst_id):

		parser = reqparse.RequestParser()
		parser.add_argument('comp_name', type=list, required=True, location='json')
		args = parser.parse_args()
		res = self.s.start(args['comp_name'])
		print(self.s)
		return res

	def delete(self,inst_id):
		
		res = self.s.remove_instance(inst_id)
		print(self.s)
		return res

class ResInstanceUrdfDyn(Resource):
	def __init__(self, server):

		self.s = server

	def post(self,inst_id):
		
		parser = reqparse.RequestParser()
		parser.add_argument('data', type=str, required=True, location='json')
		args = parser.parse_args()
		res = self.s.update_instance_urdf(inst_id ,args['data'])
		print(self.s)
		return res

class ResAvailComps(Resource):

	def __init__(self, server):
		
		self.s = server

	def get(self):
		
		res = self.s.get_avail_comps()
		print(self.s)
		return res

	def post(self):
		#add mutiple avail res

		parser = reqparse.RequestParser()
		parser.add_argument('components', type=list, required=True, location='json')
		args = parser.parse_args()

		self.s.add_comps(args['components'])
		res = self.s.get_avail_comps() 
		print(self.s)
		return res


class ResAvailComp(Resource):

	def __init__(self, server):
		self.s = server

	def get(self,name):
		print(self.s)
		return  self.s.get_avail_comp(name)

	def delete(self,name):
		res = self.s.remove_avail_comp(name)
		print(self.s)
		return res