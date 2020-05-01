import yaml

with open('cfg_ros_comp_withour_urdf.yml','r') as f:
	cfg = yaml.load(f)

robots =['UR3', 'UR5', 'UR10']
for rob in robots:
	with open('cfg/%s.xml'%rob,'r') as f:
		data =  f.read()
	for com in cfg['components']:
		print (com)
		if com['pretty_name'] == rob:
			com['urdf']['stat'] = data 
			com['urdf']['dyn'] = data

with open('cfg_ros_comp.yml','w') as f:
	yaml.dump(cfg, f)