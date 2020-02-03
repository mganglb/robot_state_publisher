# Markus Ganglbauer

from moya.common_roslaunch import create_param
from moya.common_roslaunch import add_prefix

### RSP launch ###
def create_params_rsp(p_robot_description, p_publish_frequency = '50.0', p_use_tf_static = 'true', p_ignore_timestamp = 'false',
                        p_robot_description_filepath = '', p_xacro_argument_list = [''], prefix = ''):
    d_robot_description = 'robot_description (string) - The original description of the robot in URDF form. This must be set at robot_state_publisher startup time, or the node will fail to start. Updates to this parameter will be reflected in the robot_description topic.'
    d_publish_frequency = 'publish_frequency (double) - The frequency at which fixed transforms will be republished to the network. Defaults to 50.0.'
    d_use_tf_static = 'use_tf_static (bool) - Whether to publish fixed joints on the static broadcaster (/tf_static topic) or on the dynamic one (/tf topic). Defaults to true, so it publishes on the /tf_static topic.'
    d_ignore_timestamp = 'ignore_timestamp (bool) - Whether to accept all joint states no matter what the timestamp (true), or to only publish joint state updates if they are newer than the last publish_frequency (false). Defaults to false.'
    
    robot_description, declare_robot_description_cmd = create_param('robot_description', p_robot_description, d_robot_description, prefix)
    publish_frequency, declare_publish_frequency_cmd = create_param('publish_frequency', p_publish_frequency, d_publish_frequency,prefix)
    use_tf_static, declare_use_tf_static_cmd = create_param('use_tf_static', p_use_tf_static, d_use_tf_static, prefix)
    ignore_timestamp, declare_ignore_timestamp_cmd = create_param('ignore_timestamp', p_ignore_timestamp, d_ignore_timestamp, prefix)
    robot_description_filepath, declare_robot_description_filepath_cmd = create_param('robot_description_filepath', p_robot_description_filepath, prefix)
    xacro_argument_list, declare_xacro_argument_list_cmd = create_param('xacro_argument_list', p_xacro_argument_list, prefix)

    parmas_dict = {add_prefix(prefix,'robot_description') : robot_description,
                add_prefix(prefix,'publish_frequency') : publish_frequency,
                add_prefix(prefix,'use_tf_static') : use_tf_static,
                add_prefix(prefix,'ignore_timestamp') : ignore_timestamp,
                add_prefix(prefix,'robot_description_filepath') : robot_description_filepath,
                add_prefix(prefix,'xacro_argument_list') : xacro_argument_list}

    action_list = [declare_robot_description_cmd, declare_publish_frequency_cmd, declare_use_tf_static_cmd, 
                    declare_ignore_timestamp_cmd, declare_robot_description_filepath_cmd, declare_xacro_argument_list_cmd]
    return parmas_dict, action_list


def get_prefix_rsp():
    return 'rsp'
