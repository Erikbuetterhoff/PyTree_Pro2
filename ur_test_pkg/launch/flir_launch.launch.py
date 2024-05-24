import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',  # Name des Pakets
            executable='usb_cam_node_exe',  # Name des ausführbaren Knotens
            name='flir_usb_cam',  # Name des Knotens im ROS-Graph
            output='screen',
            parameters=[
                {'video_device': '/dev/video4'},  # Pfad zum Video-Gerät
                {'image_width': 160},  # Bildbreite
                {'framerate': 9.0},  # Framerate
                {'image_height': 120},  # Bildhöhe
                {'pixel_format': 'uyvy'},  # Pixelformat
                {'camera_frame_id': 'usb_cam_frame'},  # Frame-ID der Kamera
                {'camera_info_url': ''},  # Deaktiviert die Kalibrierungsdatei
            ],
        ),
    ])

if __name__ == '__main__':
    generate_launch_description()
