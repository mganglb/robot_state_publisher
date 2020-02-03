import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory # from launch_ros.substitutions import FindPackageShare

from robot_state_publisher.common_roslaunch import create_params_rsp, get_prefix_rsp

from moya.common_roslaunch import create_common_params_node_adv, add_action_list_to_launch_description, create_conditioned_node
from moya.common_roslaunch import param_dict_to_param_list, create_common_node_argument_list, add_prefix, get_common_dirs
from moya.common_ros2 import xacro_to_urdf 


def generate_launch_description():
    ld = LaunchDescription()
    pkg_dir_rsp, launch_dir_rsp, param_dir_rsp, rviz_dir_rsp, data_dir_rsp, urdf_dir_rsp = get_common_dirs('robot_state_publisher')

    launchfile_rsp = os.path.join(launch_dir_rsp, 'rsp_upload.launch.py')
    robot_description_xacrofile = os.path.join(urdf_dir_rsp, 'test-desc.urdf.xacro')

    xacro_argument_list = ['']
    robot_description_urdf, xacro_argument_list = xacro_to_urdf(robot_description_xacrofile, xacro_argument_list)
 
    # common defaults rsp
    p_use_node_rsp = 'True'                  
    p_namespace_rsp = ""
    p_use_namespace_rsp = 'False'
    p_use_sim_time_rsp = 'False'
    p_nodename_rsp = 'robot_state_publisher'
    p_use_nodename_rsp = 'True' 
    p_logging_level = 'info'   
    prefix_rsp = get_prefix_rsp()       

    # defaults variables rsp 
    p_robot_description_rsp = robot_description_urdf
    p_publish_frequency_rsp = '50.0'
    p_use_tf_static_rsp = 'True'
    p_ignore_timestamp_rsp = 'False'
    p_robot_description_filepath_rsp = robot_description_xacrofile
    p_xacro_argument_list_rsp = xacro_argument_list

    # LaunchConfiguration common rsp
    common_param_dict_rsp, common_action_list_rsp = create_common_params_node_adv(p_use_node = p_use_node_rsp, 
        p_namespace = p_namespace_rsp, p_use_namespace = p_use_namespace_rsp,  
        p_nodename = p_nodename_rsp, p_use_nodename = p_use_nodename_rsp, 
        p_logging_level = p_logging_level, p_use_sim_time = p_use_sim_time_rsp, 
        prefix = prefix_rsp)
    add_action_list_to_launch_description(ld, common_action_list_rsp)

    # LaunchConfiguration rsp  
    param_dict_rsp, action_list_rsp = create_params_rsp(p_robot_description_rsp, 
                        p_publish_frequency = p_publish_frequency_rsp, 
                        p_use_tf_static = p_use_tf_static_rsp, 
                        p_ignore_timestamp = p_ignore_timestamp_rsp, 
                        p_robot_description_filepath = p_robot_description_filepath_rsp, 
                        p_xacro_argument_list = p_xacro_argument_list_rsp,
                        prefix = prefix_rsp)
    add_action_list_to_launch_description(ld, action_list_rsp)

    param_list_rsp = param_dict_to_param_list({**common_param_dict_rsp, **param_dict_rsp}, prefix_rsp)
    arguments_list_rsp = create_common_node_argument_list(common_param_dict_rsp[add_prefix(prefix_rsp, "logging_level")])
    remappings_rsp = []

    node_list_rsp = create_conditioned_node('robot_state_publisher',
                                            'robot_state_publisher',
                                            common_param_dict_rsp[add_prefix(prefix_rsp,"nodename")],
                                            common_param_dict_rsp[add_prefix(prefix_rsp, "namespace")],
                                            parameters = param_list_rsp,
                                            arguments = arguments_list_rsp,
                                            remappings = remappings_rsp,
                                            condition_use_namespace = common_param_dict_rsp[add_prefix(prefix_rsp, "use_namespace")],
                                            condition_use_node = common_param_dict_rsp[add_prefix(prefix_rsp, "use_node")],
                                            condition_use_nodename = common_param_dict_rsp[add_prefix(prefix_rsp, "use_nodename")])

    add_action_list_to_launch_description(ld, node_list_rsp)
    return ld