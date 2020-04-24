import requests
import json
import logging
import coloredlogs

logger = logging.getLogger(__name__)
coloredlogs.install(level='INFO')

def get(ip):
	logger.info('GET: '+ip)
	resp = requests.get(ip)
	if resp.status_code != 200:
		# This means something went wrong.
		print('GET /tasks/ {}'.format(resp.status_code))
	else:
		response = resp.json()
		print (json.dumps(response, sort_keys=True, indent=2))

def post(ip,json_data):
	logger.info('POST: '+ip)
	resp = requests.post(ip, json=json_data)
	if resp.status_code != 200:
		print("ERROR")
		print('POST /tasks/ {}'.format(resp.status_code))
	response = resp.json()
	print (json.dumps(response, sort_keys=True, indent=2))

ip = 'http://127.0.0.1:5000/'

get(ip+'AvailComps')
get(ip+'AvailComps/UR3')

get(ip+'Instances')


wd = '/home/jonas/Documents/Repos/3D_Vision_AR_RobotVis/AR_Manager/src/'
with open('src/client/two_avail_comps.json') as json_file:
	data = json.load(json_file)
	
print('\n\n\n\n\n',data)
post(ip+'AvailComps',data)

with open('src/client/start_inst.json') as json_file:
	data = json.load(json_file)
	print(data)
post(ip+'Instances',data)
post(ip+'Instances',data)

data = {'data': "best urdf string ever"}
post(ip+'Instances/1/inst/urdf_dyn',data)
