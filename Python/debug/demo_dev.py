import time
import roslibpy
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
import requests
import json
from pynput.keyboard import Key, Listener
from rest_keyboard_interface import RestKeyboardListener

import base64
import matplotlib
#from matplotlib import plt
import matplotlib.pyplot as plt
from math import pi
import queue

from io import BytesIO


from PIL import Image


ip ='http://vservice-host-001.chera.ch' #127.0.0.1
ip_ros ='vservice-host-001.chera.ch' 
port_rest = 5000
port_rossharp = 9090
import numpy
 
class Plot ():
	def __init__(self, line_cfgs):
		"""
		provide a list of line_names
		"""
		self.fig_joint = plt.figure(dpi=30)
		self.ax_joint = self.fig_joint.add_subplot(111)
		self.ax_joint.set_ylim(-pi,pi)
		self.ax_joint.set_xlim(0,101)
		self.ax_joint.set_ylabel("Radians")
		self.lines = []
		for line in line_cfgs:
			tmp, = self.ax_joint.plot([],line['color'],label=line['name'])
			self.lines.append(tmp)

		#self.ax_joint.legend()
		self.fig_joint.canvas.draw()
		self.axbackground = self.fig_joint.canvas.copy_from_bbox(self.ax_joint.bbox)
		s, (width, height) = self.fig_joint.canvas.print_to_buffer()
		self.w = width
		self.h = height
		#plt.show(block=False)

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
		#im = fig2img(self.fig_joint)
		def fig2rgb_array(fig):
			buf = self.fig_joint.canvas.tostring_rgb() 
			return np.fromstring(buf, dtype=np.uint8).reshape(self.h, self.w, 3)

		#print(self.fig_joint.canvas.get_width_height())
		buf = fig2rgb_array(self.fig_joint)

		im = Image.fromarray(buf)
		#print(self.fig_joint.canvas.get_width_height())
		#im = Image.frombytes('RGB', self.fig_joint.canvas.get_width_height(),self.fig_joint.canvas.tostring_rgb())
		img = BytesIO()
		im.save(img, "PNG")
		#plt.savefig(img, format='png', dpi=50,quality=1)
		#img.seek(0)
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

def base64_string( img ):
	img_b64 = base64.b64encode(img.getvalue()).decode()
	msg = {'data': img_b64}
	return msg

def create_ModelState(data):
	model_name = 'robot'
	pose = create_Pose([0,0,0],[0,0,0,1])
	twist = {'linear':{'x':0,'y':0,'z':0}, 'angular':{'x':0,'y':0,'z':0}}
	reference_frame = 'reference_frame'
	return {
		'model_name':model_name,
		'pose':pose,
		'twist':twist,
		'reference_frame':reference_frame
	}

   

ros = roslibpy.Ros(host=ip_ros, port= port_rossharp)
ros.run()
xyz = []

talker = roslibpy.Topic(ros, '/euclidean_goal_pose', 'geometry_msgs/Pose')

talker_img = roslibpy.Topic(ros, '/img/base_joint_speed', 'String')
talker_img2 = roslibpy.Topic(ros, '/img/base_joint_speed2', 'String')

listener = roslibpy.Topic(ros, '/joint_states', 'sensor_msgs/JointState')
talker_gazebo_reset = roslibpy.Topic(ros, '/gazebo/set_model_state', 'gazebo_msgs/ModelState')


joint_postions = [queue.Queue(maxsize=100)]

#listener.subscribe(lambda message: print('\n \n Joints:' +str(message['name'])+'\nStates:'+ str(message['position'])+ '\n Goal:' +str(xyz) ) )

from functools import partial

def got_topic (message, jp ):
	#print(message)
	if jp[0].full():
		jp[0].get()
	jp[0].put(message['position'])


listener.subscribe( partial(got_topic, jp = joint_postions) )

jointPlot = Plot([{'name':'JointAngle','color':'r'} ,{'name':'JointGoal','color':'b'}])
jointPlot2 = Plot([{'name':'JointAngle','color':'r'} ,{'name':'JointGoal','color':'b'}])



x =  range(0,100)
y1 = range(0,100)
y2 = range(10,110)
jointPlot3 = Plot([{'name':'JointAngle','color':'r'}])
jointPlot3.update(x, [y1,y2])
img3 = jointPlot3.image_png()
msg3 = base64_string(img3)
print("This workS")

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
	t_start = time.time()
	jp_list = np.array(list(joint_postions[0].queue))
	#print('pjp', jp_list)
	y1 = jp_list[:,0].tolist()
	y2 = jp_list[:,1].tolist()
	y3 = jp_list[:,2].tolist()
	y4 = jp_list[:,3].tolist()
	x =  range(0,len(y1))
	random_col = np.random.randint(0,255)
	i += 0.1
	xyz = [math.sin(i)*amp,math.sin(i)*amp,math.sin(i)*amp]
	msg =  create_Pose( xyz, [0,0,0] ) 
	talker.publish(roslibpy.Message( msg ) )
	#print( msg )

	#talker_gazebo_reset.publish( roslibpy.Message( msg )   )
	if len(y1) == 100:
		print("TIME1:", time.time()-t_start)
		t_inter = time.time()
		jointPlot.update(x, [y1,y2])
		jointPlot2.update(x, [y3,y4])
		img = jointPlot.image_png()
		img2 = jointPlot2.image_png()

		msg = base64_string(img)
		msg2 = base64_string(img2)
		
		talker_img.publish(  roslibpy.Message( msg ) )
		talker_img2.publish(  roslibpy.Message( msg2 ) )
		print("TIME2:", time.time()-t_inter)
	#np_img_data = np.ones( (200,200,3), dtype= np.uint8)*random_col 
	#print(type(img))
	#talker_img.publish()
	#msg = base64_string(np_img_data)
	#
	#print('Position send' + str(msg))
	#test_img = Image.fromarray(np_img_data)
	#test_img.show()

	time.sleep(0.01)

talker.unadvertise()
ros.terminate()



