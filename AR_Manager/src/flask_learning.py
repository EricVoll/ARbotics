print("starting")
from flask import Flask
from flask_restful import Resource, Api
 
app = Flask(__name__)
api = Api(app)

Data=[]

class People(Resource):

	def get(self,name):
		for x in Data:
			if x['Data'] == name:
				return x
		return {'Data':None}

	def post(self,name):
		Temp = {'Data':name}
		Data.append(Temp)
		return Temp
		

	def delete(self,name):

		for ind, x in enumerate(Data):
			if x['Data'] == name:
				Temp = Data.pop(ind)
				return {'Note':"Deleted"}


# class HelloWorld(Resource):
# 	def __init__(self):
# 		pass

# 	def get(self):
# 		return{
# 			"Hello":"World"
# 		}



#api.add_resource(HelloWorld,'/')
api.add_resource(People,'/Name/<string:name>')

if __name__ == '__main__':
	app.run(debug=True) 