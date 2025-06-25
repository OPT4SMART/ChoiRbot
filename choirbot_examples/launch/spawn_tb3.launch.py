from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ros_gz_bridge.actions import RosGzBridge
import yaml

def robot_entities(context, *args, **kwargs):
    id = int(LaunchConfiguration('id').perform(context))
    x_pose = float(LaunchConfiguration('x_pose').perform(context))
    y_pose = float(LaunchConfiguration('y_pose').perform(context))
    
    xacro = FindExecutable(name="xacro")
    sdf_xacro = PathJoinSubstitution([FindPackageShare('choirbot_examples'), "tb3_ns.sdf.xacro"])
    
    # build sdf from xacro
    robot_desc  = Command([xacro, " ", sdf_xacro, f" ns:=agent_{id}", f" export_rsp:=false"])
    robot_desc_rsp  = Command([xacro, " ", sdf_xacro, f" ns:=agent_{id}", f" export_rsp:=true"])

    # robot state publisher
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        namespace=f'agent_{id}',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_desc_rsp,
            # 'frame_prefix': PythonExpression(["'", f'agent_{id}', "/'"]),
            # 'publish_frequency': 50.0,
        }],
    )

    # turtlebot spawner
    spawn_turtlebot_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        parameters=[{
            # 'file': sdf_xacro,
            'string': robot_desc,
            # 'topic': f'agent_{id}/robot_description',
            'name': f'agent_{id}',
            'x': x_pose,
            'y': y_pose,
            'z': 0.01,
        }],
    )

    # bridge parameters file
    bridge_config_path = PathJoinSubstitution([
        FindPackageShare('choirbot_examples'),
        'turtlebot3_' + EnvironmentVariable('TURTLEBOT3_MODEL').perform(context) + '_bridge.yaml'
    ]).perform(context)
    with open(bridge_config_path, 'r') as yaml_file:
        yaml_data = yaml.safe_load(yaml_file)

    # modify yaml fields
    for topic in yaml_data:
        if 'ros_topic_name' in topic:
            topic['ros_topic_name'] = f"agent_{id}/{topic['ros_topic_name']}"
        if 'gz_topic_name' in topic:
            topic['gz_topic_name'] = f"model/agent_{id}/{topic['gz_topic_name']}"

    # write new bridge yaml config file
    output_yaml_path = bridge_config_path[:-5] + f'_agent_{id}.yaml'
    with open(output_yaml_path, 'w') as yaml_file:
        yaml.dump(yaml_data, yaml_file)

    # ROS gazebo agnet topic bridge
    rosgz_bridge_cmd = RosGzBridge(
        bridge_name=f'bridge_agent_{id}',
        config_file=output_yaml_path,
    )
    
    entities = [
        robot_state_publisher_cmd,
        spawn_turtlebot_cmd,
        rosgz_bridge_cmd
    ]
    return entities

def generate_launch_description():
    return LaunchDescription([
        # declare launch arguments
        DeclareLaunchArgument(
            'id',
            default_value='0',  
            description='agent_id to substitute when spawning tb3 inside gazebo'
        ),
        DeclareLaunchArgument(
            'x_pose',           
            default_value='0.0',   
            description='x position to spawn tb3 inside gazebo'
        ),
        DeclareLaunchArgument(
            'y_pose',            
            default_value='0.0',    
            description='y position to spawn tb3 inside gazebo'
        ),
        
        # build the nodes at runtime
        OpaqueFunction(function=robot_entities)
    ])