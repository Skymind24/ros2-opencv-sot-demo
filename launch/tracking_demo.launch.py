import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, EmitEvent, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart, OnExecutionComplete, OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.substitutions import FindExecutable, LaunchConfiguration, EnvironmentVariable, LocalSubstitution

def generate_launch_description():

    package_name='ros2_opencv_sot_demo' # CHANGE ME

    ### Lanch File ###
    camera = IncludeLaunchDescription(
               PythonLaunchDescriptionSource([os.path.join(
                   get_package_share_directory(package_name),'launch','camera.launch.py'
               )])
    )

    ### Node ###
    tracker_node = Node(
        package=package_name,
        executable='tracker',
        remappings=[
            ('/image_raw', 'image_raw'),
        ],
        output='screen'
    )

    detector_node = Node(
        package=package_name,
        executable='detector',
        remappings=[
            ('/image_raw', 'image_raw'),
            ('/detection/detector', 'detection/detector')
        ],
        output='screen'
    )

    ### ExecuteProcess ###
    # execute_detector = ExecuteProcess(
    #     cmd=[[
    #         FindExecutable(name='ros2'),
    #         ' run ',
    #         package_name,
    #         detector_node
    #     ]],
    #     shell=True
    # )

    # execute_tracker = ExecuteProcess(
    #     cmd=[[
    #         FindExecutable(name='ros2'),
    #         ' run ',
    #         package_name,
    #         tracker_node
    #     ]],
    #     shell=True
    # )

    rqt = ExecuteProcess(
        cmd=["rqt"], 
        output="screen",
        shell=True
    )

    ### Event Handlers ###
    # start_detect = RegisterEventHandler(
    #     OnProcessStart(
    #         target_action=camera,
    #         on_start=[
    #             LogInfo(msg='Camera started, detecting the target'),
    #             execute_detect
    #         ]
    #     )
    # )

    # start_track = RegisterEventHandler(
    #     OnExecutionComplete(
    #         target_action=detector_node,
    #         on_completion=[
    #             LogInfo(msg='Detect finished'),
    #             tracker_node
    #         ]
    #     )
    # )
    


    return LaunchDescription([

        camera,
        detector_node,
        tracker_node,
        rqt,
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[LogInfo(
                    msg=['Launch was asked to shutdown: ',
                        LocalSubstitution('event.reason')]
                )]
            )
        ),
    ])
