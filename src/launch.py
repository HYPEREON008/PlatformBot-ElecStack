from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

    Node(
        package = 'driver',
        executable = 'motor_driver',

    ),

    Node(
        package = 'driver',
        executable = 'plot_monitor',

    ),


    Node(
        package = 'driver',
        executable = 'encoder_serial',

    ),



    Node(
        package = 'driver',
        executable = 'pid_node',

    ),

    ])
