import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # Start the V4L2 camera node
        launch_ros.actions.Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='camera',
            parameters=[{'image_size': [640, 480]}],
            remappings=[('/image_raw', '/camera/image_raw')]
        ),
        
        # Start image transport to publish compressed images
        launch_ros.actions.Node(
            package='image_transport',
            executable='republish',
            name='image_republisher',
            arguments=['raw', 'compressed'],
            remappings=[('/in', '/camera/image_raw'), ('/out/compressed', '/camera/image_compressed')]
        ),
    ])
