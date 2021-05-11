
from launch import LaunchDescription


def generate_launch_description():
    bagsaver = Node(
        package="report_utils",
        # namespace="/",
        executable="bagsaver",
        name="bagsaver",
        arguments=[
            "--bagsaver-config", "launch/bagsaver_config.yaml"
        ],
    )

    ld = LaunchDescription([
        bagsaver
    ])

    return ld
