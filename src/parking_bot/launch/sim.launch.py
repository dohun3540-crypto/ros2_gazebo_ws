import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('parking_bot')

    xacro_file = os.path.join(pkg_share, 'urdf', 'parking_bot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'parking_lot.sdf')
    bridge_config = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # gz-sim 이 world 파일 안의 world 이름을 찾을 수 있도록 리소스 경로 등록
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(pkg_share, '..')
    )

    # 렌더엔진 시행착오 기록 (worlds/parking_lot.sdf 상단 주석과 함께 참고):
    # - ogre2 + D3D12 GPU 가속: GUI/센서 모두 GL3Plus GenerateHwMipmaps에서
    #   Ogre::UnimplementedException abort. 100% 재현됨.
    # - ogre1(legacy) + D3D12: 크래시는 없지만 gpu_lidar가 로봇 자기 몸체를
    #   오탐지해 스폰 직후부터 emergency-avoidance가 계속 발동, SEARCH가 아예
    #   실행이 안 되고 장애물에 충돌.
    # - ogre2 + 완전 소프트웨어 렌더링(LIBGL_ALWAYS_SOFTWARE=1, GPU 강제 없음):
    #   이전 세션 로그(parking_run2~5.log 등)에서 여러 번 안정적으로 SEARCH부터
    #   FINISHED까지 정상 동작 확인됨 - 현재 채택.
    force_libgl_software = SetEnvironmentVariable(
        name='LIBGL_ALWAYS_SOFTWARE', value='1'
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    # 주차구역 앞 차선(아일) 위에서 +x 방향으로 직진하며 스폰한다. 장애물 행은
    # y>0 쪽에 있으므로(로봇 기준 왼쪽, local +90도), y=-0.6으로 충분히 떨어뜨려
    # 둔다 - 이 여유가 없으면 나중에 ROTATE_IN_PLACE(제자리 회전)에서 차체
    # 대각선 반경(약 0.8m)이 옆 장애물에 스치기 때문(parking_controller.py의
    # ROTATE_IN_PLACE 관련 주석 참고). x=-2.0은 첫 번째 장애물(parked_box_left,
    # x=[-1.1,-0.6])에 도달하기 전 직진 상태를 안정시킬 여유 구간이다.
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'parking_bot',
            '-x', '-2.0', '-y', '-0.6', '-z', '0.05',
            '-Y', '0.0',
        ],
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': use_sim_time,
        }],
    )

    parking_controller = Node(
        package='parking_bot',
        executable='parking_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        set_gz_resource_path,
        force_libgl_software,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        parking_controller,
    ])
