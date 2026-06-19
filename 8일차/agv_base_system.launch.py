from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo

def generate_launch_description():
    
    # 1. 하드웨어 및 오도메트리 실행 (터미널 명령어와 동일)
    odometry_process = ExecuteProcess(
        cmd=['ros2', 'launch', 'myagv_odometry', 'myagv_active.launch.py'],
        output='screen'
    )

    # 2. 내비게이션 및 맵 실행 (터미널 명령어와 완전히 동일하게 맵 경로 전달)
    navigation_process = ExecuteProcess(
        cmd=['ros2', 'launch', 'myagv_navigation2', 'navigation2_active.launch.py', 'map:=/home/er/my_map.yaml'],
        output='screen'
    )

    # ★ 핵심: 하드웨어와 센서가 켜질 시간을 5초 벌어준 뒤에 내비게이션 맵을 켭니다.
    delayed_navigation = TimerAction(
        period=5.0,
        actions=[
            LogInfo(msg="[시스템] 하드웨어 가동 완료. 5초 대기 끝! 지정된 맵으로 내비게이션을 시작합니다."),
            navigation_process
        ]
    )

    return LaunchDescription([
        odometry_process,
        delayed_navigation
    ])