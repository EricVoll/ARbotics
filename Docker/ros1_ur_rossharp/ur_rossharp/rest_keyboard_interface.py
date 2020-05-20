from pynput.keyboard import Key, Listener

import requests
import json
# rest requests
def get(ip, log = False):
	logger.info('GET: '+ip)
	resp = requests.get(ip)
	if resp.status_code != 200:
		# This means something went wrong.
		print('GET /tasks/ {}'.format(resp.status_code))
	else:
		response = resp.json()
		if log:
			print (json.dumps(response, sort_keys=True, indent=2))
	return response

def post(ip,json_data, log = False):
	"""
	Inputs: 
	ip = IP-Adress of Server
	json_data: Dict 
	log: Flag if received data is printed
	"""
	logger.info('POST: '+ip)
	resp = requests.post(ip, json=json_data)
	if resp.status_code != 200:
		print("ERROR")
		print('POST /tasks/ {}'.format(resp.status_code))
	response = resp.json()
	if log:
		print (json.dumps(response, sort_keys=True, indent=2))
	
	return response
print("hello")

ip_adr = 'http://127.17.0.1:5000/'
s_pose = """<joint name="wrist_3_joint" type="revolute">
    <parent link="wrist_2_link"/>
    <child link="wrist_3_link"/>
    <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.09465"/>
    <axis xyz="0 1 0"/>
    <limit effort="28.0" lower="-3.14159265359" upper="3.14159265359" velocity="3.2"/>
    <dynamics damping="0.0" friction="0.0"/>
  </joint>"""

def insert_str(string, index, str_to_insert):
    return string[:index] + str_to_insert + string[index:]

add_string_to_xml = """<tooltip name="base_joint_speed" topic="/img/base_joint_speed">
	  <parent link="wrist_1_joint"/>
  </tooltip>
  """


# keyboard listener
def on_press(key):
	#pass
  print('{0} pressed'.format(
       key))

def on_release(key):
	if key == 's':
		print("s pressed")
		instances = get(ip_adr+'Instances')
		print(instances)
		for index, ins in enumerate(instances):
			if ins['comp']['pretty_name'] == 'UR5':
				i=index
				urdf_dyn = ins['comp']['urdf_stat']
				pose = urdf_dyn.fing(s_pose)
				out = insert_str(urdf_dyn, pose, add_string_to_xml)

		#modified URDF with tooltip
		data = {'data': out}
		print(out)
		post(ip+'Instances/%d/inst/urdf_dyn'%i,data)


	elif key == 'd':
		print("d pressed")
		instances = get(ip_adr+'Instances')
		print(instances)
		for index, ins in enumerate(instances):
			if ins['comp']['pretty_name'] == 'UR5':
				i=index
				urdf_stat = ins['comp']['urdf_stat']
		#reset urdf_dyn to stat
		data = {'data': out}
		print(out)
		post(ip+'Instances/%d/inst/urdf_dyn'%i,data)

	elif key == Key.esc:
			# Stop listener
			return False
	return True



def RestKeyboardListener():
	
	return Listener(
			on_press=on_press,
			on_release=on_release)
