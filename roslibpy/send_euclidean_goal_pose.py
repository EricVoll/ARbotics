import time
import roslibpy
import numpy as np
from scipy.spatial.transform import Rotation as R
import math

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

ros = roslibpy.Ros(host='127.0.0.1', port=9090)
ros.run()
xyz = []

talker = roslibpy.Topic(ros, '/euclidean_goal_pose', 'geometry_msgs/Pose')
listener = roslibpy.Topic(ros, '/joint_states', 'sensor_msgs/JointState')
listener.subscribe(lambda message: print('\n \n Joints:' +str(message['name'])+'\nStates:'+ str(message['position'])+ '\n Goal:' +str(xyz) ) )

i = 0
amp = 0.4
while ros.is_connected:
	i += 0.1
	xyz = [math.sin(i)*amp,math.sin(i)*amp,math.sin(i)*amp]
	msg =  create_Pose( xyz, [0,0,0] ) 
	talker.publish(roslibpy.Message( msg ) )
	#print('Position send' + str(msg))
	time.sleep(1)
talker.unadvertise()
ros.terminate()



