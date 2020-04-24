import sys
sys.path.append('/home/jonas/Documents/Repos/3D_Vision_AR_RobotVis/AR_Manager/src')
sys.path.append('/home/jonas/Documents/Repos/3D_Vision_AR_RobotVis/AR_Manager/')

from flask_restful import Resource, reqparse

class ResInstances(Resource):
	def __init__(self, server):
		self.s = server

	def get(self):
		data = self.s.get_instances()
		print(data)
		return data #returns the id of the instance

	def post(self):
		print(self.s)
		parser = reqparse.RequestParser()
		parser.add_argument('comp_name', type=str, required=True, location='json')
		args = parser.parse_args()

		return self.s.start(args['comp_name'])
	def delete(self):
		#delete all runing comps
		print(self.s)
		self.s.stop_instances()
		return {'Suc':True}
	
class ResInstance(Resource):
	def __init__(self, server):
		self.s = server

	def get(self,inst_id):
		print(self.s)
		return self.s.get_instance(inst_id)

	
	def post(self,inst_id):
		print(self.s)
		parser = reqparse.RequestParser()
		parser.add_argument('comp_name', type=list, required=True, location='json')
		args = parser.parse_args()

		return self.s.start(args['comp_name'])

		
	def delete(self,inst_id):
		return self.s.remove_instance(inst_id)

class ResAvailComps(Resource):

	def __init__(self, server):
		self.s = server

	def get(self):
		print(self.s)
		return self.s.get_avail_comps()
		
	def post(self):
		print(self.s)
		#add mutiple avail res

		parser = reqparse.RequestParser()
		parser.add_argument('components', type=list, required=True, location='json')
		args = parser.parse_args()
		print("input                    ",len( args['components']) , type(args['components']) , args['components'])
		self.s.add_comps(args['components'])
		res = self.s.get_avail_comps() 
		print(self.s)
		return res


class ResAvailComp(Resource):

	def __init__(self, server):
		self.s = server

	def get(self,name):
		print(self.s)
	
		return self.s.get_avail_comp(name) 
	
	# def post(self,name):
	# 	#add mutiple avail res
	# 	return {'instance_id': 100} #returns the id of the instancep
			
	def delete(self,name):
		#delete avail 
		res = self.s.remove_avail_comp(name)
		print(self.s)
		return res