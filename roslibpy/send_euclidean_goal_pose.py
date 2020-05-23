import time
import roslibpy
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
import requests
import json
from pynput.keyboard import Key, Listener
from rest_keyboard_interface import RestKeyboardListener
from PIL import Image
import base64


ip ='http://vservice-host-001.chera.ch' #127.0.0.1
ip_ros ='vservice-host-001.chera.ch' 
port_rest = 5000
port_rossharp = 9090

class Plot ():
	def __init__(self, line_cfgs):
		"""
		provide a list of line_names
		"""
		self.fig_joint = plt.figure()
		self.ax_joint = self.fig_joint.add_subplot(111)
		self.ax_joint.set_ylim(-pi,pi)
		self.ax_joint.set_xlim(0,101)
		self.ax_joint.set_ylabel("Radians")
		self.lines = []
		for line in line_cfgs:
			tmp, = self.ax_joint.plot([],line['color'],label=line['name'])
			self.lines.append(tmp)

		self.ax_joint.legend()
		self.fig_joint.canvas.draw()
		self.axbackground = self.fig_joint.canvas.copy_from_bbox(self.ax_joint.bbox)
		
		plt.show(block=False)

	def update(self,x, ys):
		"""
		Input x is array of values for x-Axis
		ys is an list of arrays containing y values with same length as x
		"""
		if not (len(x) == len(ys[0])):
			raise TypeError()

		for i, line in enumerate(self.lines):
			line.set_data(x,ys[i])

		self.fig_joint.canvas.restore_region(self.axbackground)
		for i, line in enumerate(self.lines):
			self.ax_joint.draw_artist(line)
		self.fig_joint.canvas.blit(self.ax_joint.bbox)
		self.fig_joint.canvas.flush_events()

	def image_png(self):
		img = BytesIO()
		plt.savefig(img, format='png')
		img.seek(0)
		return img



def create_MultiArray32(data):
	dim = []
	for i in range(0,len(data.shape)):
		out = 'axis'+str(i)
		dim.append( {'label': out, 'size': data.shape[i], 'stride':1 })

	return { 'layout': {'dim': dim , 'data_offset':0}, \
			'data': data.tolist() }

def create_Pose(t,xyz):
	q = R.from_euler('xyz' , xyz).as_quat()
	return { 'position': {'x':t[0], 'y':t[1], 'z':t[2]} , 'orientation':{'x':q[0] ,'y':q[1],'z':q[2],'w':q[3] } }

def create_Img(data):
	"""creates ros compatible img
	Parameters
	----------
	data : numpy.array
		width height RGB
	"""
	header = {'seq': int(10), 'time':0, 'frame_id':0}
	height = int(data.shape[0])
	width = int(data.shape[1])
	encoding = 'rgb8'
	is_bigendian = int(0)
	step = int( 1 )
	data = data.tolist()

	msg = {'header':header,
	'heihgt': height,
	'width':width,
	'encoding':encoding,
	'is_bigendian':is_bigendian,
	'step':step,
	'data':data }
	return msg

def base64_string( data ):
	img_b64 = base64.b64encode(img.getvalue()).decode()
	msg = {'data': img_b64}
	return msg


# def create_ModelState(data):
#  	model_name = 'robot'
# 	pose = create_Pose()
# 	geometry_msgs/Pose pose
# 	geometry_msgs/Twist twist
# 	string reference_frame

ros = roslibpy.Ros(host=ip_ros, port= port_rossharp)
ros.run()
xyz = []


talker = roslibpy.Topic(ros, '/euclidean_goal_pose', 'geometry_msgs/Pose')
talker_img = roslibpy.Topic(ros, '/img/base_joint_speed', 'String')

listener = roslibpy.Topic(ros, '/joint_states', 'sensor_msgs/JointState')
talker_gazebo_reset = roslibpy.Topic(ros, '/gazebo/set_model_state', 'gazebo_msgs/ModelState')

#listener.subscribe(lambda message: print('\n \n Joints:' +str(message['name'])+'\nStates:'+ str(message['position'])+ '\n Goal:' +str(xyz) ) )
# jointPlot = Plot2([{'name':'JointAngle','color':'r'} ,{'name':'JointGoal','color':'b'}])


i = 0
amp = 0.4
keylis = RestKeyboardListener(ip_adr=ip, port_rest=str(port_rest) )
keylis.start()
import os
def menu():

	os.system('cls' if os.name == 'nt' else 'clear')
	l = 6
	l2 = 74
	print(''.ljust(l,'-') + '+'.ljust(l2,'-')+'+')
	print('Key:'.ljust(l, ' ') + '| Action:'.ljust(l2, ' ') + '|' )
	print(''.ljust(l,'-') + '+'.ljust(l2,'-')+'+')
	print('s'.ljust(l, ' ') + '| Adds Tooltip to UR5-Instance Dyn-URDF via AR-Manager REST-Request'.ljust(l2, ' ') + '|' )
	print('d'.ljust(l, ' ') +'| Resets UR5-Instance Dyn-URDF via AR-Manager REST-Request'.ljust(l2, ' ') + '|' )


while ros.is_connected:
	#menu()
	random_col = np.random.randint(0,255)
	i += 0.1
	xyz = [math.sin(i)*amp,math.sin(i)*amp,math.sin(i)*amp]
	msg =  create_Pose( xyz, [0,0,0] ) 
	talker.publish(roslibpy.Message( msg ) )




	np_img_data = np.ones( (200,200,3), dtype= np.uint8)*random_col 
	
	
	#msg = base64_string(np_img_data)
	#talker_img.publish(  roslibpy.Message(  ) )
	#print('Position send' + str(msg))
	#test_img = Image.fromarray(np_img_data)
	#test_img.show()
	time.sleep(1)
	#test_img.close()

talker.unadvertise()
ros.terminate()



