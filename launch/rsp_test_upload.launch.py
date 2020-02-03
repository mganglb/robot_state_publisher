import os

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch_ros.actions import PushRosNamespace
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from robot_state_publisher.common_roslaunch import create_params_rsp, get_prefix_rsp

from moya.common_roslaunch import create_common_params_node_adv, create_common_params_group
from moya.common_roslaunch import add_action_list_to_launch_description, get_common_dirs
from moya.common_ros2 import xacro_to_urdf 


def generate_launch_description():
    ld = LaunchDescription()

    pkg_dir_rsp, launch_dir_rsp, param_dir_rsp, rviz_dir_rsp, data_dir_rsp, urdf_dir_rsp = get_common_dirs('robot_state_publisher')

    launchfile_rsp = os.path.join(launch_dir_rsp, 'rsp_upload.launch.py')
    robot_description_xacrofile = os.path.join(urdf_dir_rsp, 'test-desc.urdf.xacro')

    xacro_argument_list = ['']
    robot_description_urdf, xacro_argument_list = xacro_to_urdf(robot_description_xacrofile, xacro_argument_list)

    # common defaults group
    p_namespace = ""
    p_use_namespace = 'false'

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

    # LaunchConfiguration group common
    common_parma_dict_group_rsp , common_action_list_group_rsp = create_common_params_group(p_namespace = p_namespace, 
        p_use_namespace = p_use_namespace)
    add_action_list_to_launch_description(ld, common_action_list_group_rsp)

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

    # rsp Node
    push_ns = PushRosNamespace(
        condition=IfCondition(common_parma_dict_group_rsp["use_namespace"]),
        namespace=common_parma_dict_group_rsp["namespace"])

    rsp_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launchfile_rsp),
        launch_arguments={**common_param_dict_rsp, **param_dict_rsp}.items())


    rsp_group = GroupAction([push_ns, rsp_ld])
    ld.add_action(rsp_group)

    return ld