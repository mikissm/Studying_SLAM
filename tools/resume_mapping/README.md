# 저장 지도에서 이어서 매핑

저장 PCD를 기준으로 현재 body 점군을 ICP 정합하고, 고정된 `map → odom` 변환과 FAST-LIO odometry로 새 점군을 누적하는 초기 버전입니다. 저장된 PGO 그래프/FAST-LIO 내부 ikd-tree를 복원하지 않습니다. 최초 정합 후 장거리 drift를 다시 보정하거나 루프 클로저를 수행하지 않습니다. XYZ voxel map이며 intensity·관측 이력·동적 장애물 제거는 지원하지 않습니다.

## 실행

각 터미널에서 아래 환경을 먼저 설정합니다. 별도 colcon 빌드는 필요 없습니다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/ssm/slam_ws/install/setup.bash
```

1. 기존 MID360 드라이버를 실행합니다.

```bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

2. 이 모드에서는 **PGO와 localizer를 실행하지 않습니다**. `map → odom` TF 발행자가 중복됩니다. FAST-LIO만 실행합니다. 이 launch에는 RViz도 포함됩니다.

```bash
ros2 launch fastlio2 lio_launch.py
```

3. 저장 맵과 **현재 센서 body의 저장 지도상 대략적인 위치**를 지정합니다. yaw 단위는 rad입니다. 0은 예시이며 실제 위치로 바꿉니다. 로봇을 정지시킨 채 정합을 시작하세요. 이미 존재하는 output 파일과 원본 파일에는 덮어쓰지 않습니다.

```bash
/home/ssm/slam_ws/Studying_SLAM/tools/resume_mapping/run.sh --ros-args \
  -p map_path:=/home/ssm/maps/test1/map.pcd \
  -p output_path:=/home/ssm/maps/test2/map.pcd \
  -p initial_x:=0.0 -p initial_y:=0.0 -p initial_z:=0.0 -p initial_yaw:=0.0
```

저장 지도 전용 설정으로 RViz를 열면 아래 설정이 자동 적용됩니다 (기존 FAST-LIO RViz 창과 구분하세요).

```bash
rviz2 -d /home/ssm/slam_ws/Studying_SLAM/tools/resume_mapping/resume_mapping.rviz
```

수동으로 설정하려면 RViz Fixed Frame을 `map`으로 설정하고 PointCloud2 `/resume_mapping/map`을 추가합니다. QoS durability는 `Transient Local`로 설정하면 시작 시 저장 맵도 바로 보입니다. RViz **2D Pose Estimate** (`/initialpose`, frame `map`)로 초기 위치를 다시 설정할 수도 있습니다. 바닥과 센서 높이가 다르면 초기 Z도 실제 body 높이에 맞춰야 합니다. 2D Pose Estimate는 보통 Z=0이므로 CLI의 initial_z가 필요한 환경에서는 CLI를 사용합니다.

4. 초기 위치를 확인한 뒤 시작합니다. 서비스 성공 응답에는 overlap과 RMSE가 포함됩니다. ICP는 가까운 초기 위치를 필요로 하며 반복 구조에서 잘못 정합될 수 있으므로, RViz에서 기존 벽과 현재 점군의 겹침을 확인한 뒤 이동합니다. 승인 전에는 누적하지 않습니다.

```bash
ros2 service call /resume_mapping/start std_srvs/srv/Trigger '{}'
ros2 topic echo /resume_mapping/status
```

5. 이동하며 새 영역을 관측한 뒤 정지/저장합니다.

```bash
ros2 service call /resume_mapping/pause std_srvs/srv/Trigger '{}'
ros2 service call /resume_mapping/save std_srvs/srv/Trigger '{}'
```

이 모드의 저장 명령은 `/pgo/save_maps`가 아닙니다. 새 PCD를 기존 elevation 웹 도구의 **PCD 열기**로 불러올 수 있습니다. 웹 화면에 실시간 스트리밍하는 기능은 아직 포함하지 않습니다. 저장 PCD는 어느 시점이든 다음 세션의 입력으로 사용할 수 있으나 세션마다 초기 정합이 필요합니다. FAST-LIO를 재시작하면 재개 노드도 재시작해야 합니다.

## 입출력과 설정

- `/fastlio2/body_cloud` (body), `/fastlio2/lio_odom` (odom → body), 시간 차 최대 0.03초로 동기화.
- `/resume_mapping/map`: 저장 맵 + 새 점군, frame `map`.
- `/resume_mapping/status`: active/aligned/voxels/error/정합 품질 JSON.
- `resolution:=0.1`, `update_hz:=2.0`, `max_voxels:=2000000`: 점군 누적/발행 한도. 기존 voxel의 표본은 유지합니다.
- `min_overlap:=0.55`, `max_rmse:=0.15`: 초기 정합 승인 기준. 대응점 거리 0.6m. 대략 지정한 위치에서 XY ±1m (0.5m 간격), Z ±0.75m (0.375m 간격), yaw ±45도 (15도 간격)의 875개 후보를 점수화하고 최대 13개를 ICP로 보정합니다. 최종 보정량은 원래 지정 위치에서 2m/60도 이내여야 합니다. 승인 기준을 통과한 후보가 없으면 누적하지 않습니다. 지도 전체를 검색하는 전역 위치 추정은 아니며 반복 구조에서 오정합 가능성이 남습니다.
- `max_scan_age:=1.0`: 오래된 입력이면 누적을 일시정지합니다. 정합 계산 직후에는 최대 3초 동안 새 점군을 기다리며 `waiting_for_scan` 상태를 표시합니다. 이 기간에도 오래된 점군은 누적하지 않습니다. 센서 재개 후 start로 다시 시작합니다.
- 2Hz 점군을 샘플링하므로 빠른 이동에서는 누락이 생길 수 있습니다. 전체 맵 발행 비용도 있어 고속 실시간 처리를 보장하지 않습니다.
- 새 초기 위치 지정은 이미 누적된 점들을 소급 재배치하지 않습니다. 오정합 상태에서 누적했다면 원본으로 재시작합니다.

## 검증

workspace root에서:

```bash
cd /home/ssm/slam_ws
/usr/bin/python3 -m unittest discover -s Studying_SLAM/tools/resume_mapping -p 'test_geometry.py' -v
source /opt/ros/jazzy/setup.bash
source install/setup.bash
/usr/bin/python3 Studying_SLAM/tools/resume_mapping/test_ros.py
```

기하 테스트는 ICP, 좌표 합성, 실패 거부, 원본 보호와 저장 재로딩, 용량 한도의 원자성을 검증합니다. ROS 테스트는 모의 점군/odom을 발행하고 실제 ROS 서비스로 정합→누적→정지→저장을 검증합니다. 실제 MID360 주행 검증은 별도로 필요합니다.

## 누적 점군 색상 구분

전용 RViz는 원본 지도(`/resume_mapping/saved`)를 회색, 원본에 없던 voxel(`/resume_mapping/added`)을 초록색으로 표시합니다. 아래 표시 노드를 재개 매핑과 함께 실행하세요. `map_path`와 `resolution`은 매핑 노드의 입력과 일치해야 합니다. 매핑 노드 재시작 없이 실행할 수 있습니다.

```bash
source /opt/ros/jazzy/setup.bash
source /home/ssm/slam_ws/install/setup.bash
/usr/bin/python3 /home/ssm/slam_ws/Studying_SLAM/tools/resume_mapping/map_overlay.py --ros-args \
  -p map_path:=/home/ssm/maps/test1/map.pcd -p resolution:=0.1
```

초록색은 최초 저장 지도 대비 추가된 voxel이며 최신 스캔 전체가 아닙니다. 이미 채워진 voxel의 재관측은 색을 바꾸지 않습니다. 센서 잡음이나 오정합으로 기존 벽 주변에 추가된 점도 초록색으로 보일 수 있습니다. 합쳐진 지도 저장 방식은 그대로입니다.
