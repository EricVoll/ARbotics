import yaml

with open('cfg/cfg_ros_comp.yml','r') as f:
	cfg = yaml.load(f)

component = cfg['components'][2]['pretty_name']
pose = cfg['components'][2]['urdf']['stat'].find("""<joint name="wrist_3_joint" type="revolute">
    <parent link="wrist_2_link"/>
    <child link="wrist_3_link"/>
    <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.09465"/>
    <axis xyz="0 1 0"/>
    <limit effort="28.0" lower="-3.14159265359" upper="3.14159265359" velocity="3.2"/>
    <dynamics damping="0.0" friction="0.0"/>
  </joint>""")
def insert_str(string, index, str_to_insert):
    return string[:index] + str_to_insert + string[index:]
print(pose, type(cfg['components'][2]['urdf']['stat'] ))
o = insert_str('abcd',2,'xxx')
print(o)
add_string_to_xml = """<tooltip name="base_joint_speed" topic="/img/base_joint_speed">
	  <parent link="wrist_1_joint"/>
  </tooltip>
  """
out = insert_str(str(cfg['components'][2]['urdf']['stat']), int(pose), add_string_to_xml )
print(out)