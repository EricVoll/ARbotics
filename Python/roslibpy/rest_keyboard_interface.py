from pynput.keyboard import Key, Listener
import roslibpy
import requests
import json
# rest requests

s_pose = """<joint name="wrist_3_joint" type="revolute">
    <parent link="wrist_2_link"/>
    <child link="wrist_3_link"/>
    <origin rpy="0.0 0.0 0.0" xyz="0.0 0.0 0.09465"/>
    <axis xyz="0 1 0"/>
    <limit effort="28.0" lower="-3.14159265359" upper="3.14159265359" velocity="3.2"/>
    <dynamics damping="0.0" friction="0.0"/>
  </joint>"""
add_string_to_xml = """<tooltip name="joint_position" topic="/img/joint_position">
	  <parent link="wrist_1_link"/>
  </tooltip>"""
add_string_to_xml_2 = """<tooltip name="tooltip_position" topic="/img/tool_position">
	  <parent link="base_link"/>
  </tooltip>"""


def insert_str(string, index, str_to_insert):
    return string[:index] + str_to_insert + string[index:]


class RestKeyboardListener():
    """ Keyboard Listener to send REST requests.
    """
    def __init__(self, ip_adr='https://127.0.0.1', port_rest='5000', ros=None):
        self.t_status = False
        self.talker_t = roslibpy.Topic(ros, '/t', 'std_msgs/Int32')

        def get(ip, log=False):
            print("ip", ip)
            print('GET: '+ip)
            resp = requests.get(ip)
            if resp.status_code != 200:
                # This means something went wrong.
                print('GET /tasks/ {}'.format(resp.status_code))
            else:
                response = resp.json()
                if log:
                    print(json.dumps(response, sort_keys=True, indent=2))
            return response

        def post(ip, json_data, log=False):
            """
            Inputs: 
            ip: IP of server
            json_data: dict 
            log: flag if received data is printed
            """
            print('POST: '+ip)
            resp = requests.post(ip, json=json_data)
            if resp.status_code != 200:
                print("ERROR")
                print('POST /tasks/ {}'.format(resp.status_code))
            response = resp.json()
            print(response)
            if log:
                print(json.dumps(response, sort_keys=True, indent=2))

            return response

        def on_press(key):
            pass

        def on_release(key):
            if str(key) == "'t'":
                print("Status is", self.t_status)
                self.t_status = not self.t_status
                msg = {'data': int(self.t_status)}
                self.talker_t.publish(roslibpy.Message(msg))

            if str(key) == "'s'":
                instances = get(ip_adr+':'+port_rest+'/Instances', log=False)

                for index, ins in enumerate(instances):
                    if ins['comp']['pretty_name'] == 'UR5':
                        i = index
                        urdf_dyn = ins['comp']['urdf_stat']
                        pose = urdf_dyn.find(s_pose)
                        out = insert_str(urdf_dyn, pose, add_string_to_xml)

                # modified URDF with tooltip
                data = {'data': out}

                post(ip_adr+':'+port_rest+'/Instances/%d/inst/urdf_dyn' % i, data)

            if str(key) == "'f'":
                instances = get(ip_adr+':'+port_rest+'/Instances', log=False)

                for index, ins in enumerate(instances):
                    if ins['comp']['pretty_name'] == 'UR5':
                        i = index
                        urdf_dyn = ins['comp']['urdf_stat']
                        pose = urdf_dyn.find(s_pose)
                        out = insert_str(urdf_dyn, pose, add_string_to_xml_2)

                # modified URDF with tooltip
                data = {'data': out}
                post(ip_adr+':'+port_rest+'/Instances/%d/inst/urdf_dyn' % i, data)

            if str(key) == "'d'":
                instances = get(ip_adr+':'+port_rest+'/Instances')
                for index, ins in enumerate(instances):
                    if ins['comp']['pretty_name'] == 'UR5':
                        i = index
                        urdf_stat = ins['comp']['urdf_stat']
                
                # reset urdf_dyn to stat
                data = {'data': urdf_stat}
                post(ip_adr+':'+port_rest+'/Instances/%d/inst/urdf_dyn' % i, data)

            elif key == Key.esc:
                return False
            return True

        self.listener = Listener(
            on_press=on_press,
            on_release=on_release)
