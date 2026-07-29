"""
LiDAR 기반 후진주차 제어 노드 (v7, 차동구동 AMR - 차선 주행 + 편측 갭 탐지 + 후방
사각지대 대응).

실제 시나리오: 로봇은 주차구역 앞 차선(아일)을 따라 똑바로 직진하다가, 이미 주차된
차량(장애물) 사이의 빈자리를 찾으면 그 중앙에 멈추고, 개활지에서 제자리 회전으로
자세를 슬롯에 대해 수직으로 돌린 뒤, 직진으로 후진해 들어간다. 자동차식 전륜 조향이
아니라 "제자리 회전이 되는 AMR"(차동구동)이므로 Ackermann 자전거 모델 없이
geometry_msgs/Twist(linear.x, angular.z)를 직접 발행해 Gazebo DiffDrive 시스템
플러그인에 명령한다.

라이다 하드웨어 제약 (urdf 기준):
- 라이다는 차체 정중앙이 아니라 앞쪽(lidar_offset_x=0.65, 차체 앞쪽 끝 0.70m 바로
  안쪽)에 달려 있고, 스캔 범위가 전방+좌/우 250도(-125~+125도, 실제 규격 기준)다.
  정후방 110도 구간만 스캔 데이터가 없다 - 앞에 달린 센서가 제 차체 바로 뒤는 볼
  수 없다는 물리적 제약을 그대로 반영한 것이다.
- 따라서 "뒤쪽 장애물까지 남은 거리"를 라이다로 재서 후진을 멈추는 방식은 쓸 수
  없다. 대신 슬롯 규격(치수는 이미 알려진 상수: 후진주차장.png 기준 입구 y=0 ~
  안쪽 정지선 y=1.6)과 차체 규격(1.4m x 0.775m, 차량규격.png 기준)으로 "목표
  정차 위치"를 미리 계산해 두고, 오도메트리로 그 위치에 도달했는지만 확인한다
  (= 뒤를 못 보는 대신 "얼마나 왔는지"로 판단).
- 좌/우 라이다 거리는 SEARCH/BACK_TO_CENTER/RETURN_SCAN(회전 전, 아직 헤딩이
  차선 방향) 동안 "옆 장애물과의 안전거리 유지(row-hold)"에 쓴다: 헤딩(각도)만
  잡으면 몇 미터를 주행하는 동안 미세한 드리프트가 누적돼도 못 잡기 때문이다.
  회전 후 STRAIGHT_REVERSE에서는 좌우 거리차 기반 실시간 정렬을 시도했었지만
  결과가 더 나빠서 뺐다(아래 STRAIGHT_REVERSE 항목 참고) - 대신 회전 전 위치
  자체를 정확히 맞추는 쪽(RETURN_SCAN)으로 해결했다.

주차 완료 조건: 차체 전체(앞범퍼~뒷범퍼)가 슬롯(y=[0, 1.6]) 안에 들어와야 한다.
슬롯 깊이 1.6m가 차체 길이 1.4m보다 겨우 200mm 크므로, 차체 중심을 슬롯 중앙
(y=0.8, 앞뒤로 각각 100mm씩 여유)에 맞추는 것을 목표 정차 위치로 삼는다.

상태머신:
- `SEARCH` (차선 직진): 직진 헤딩 + 옆 장애물과의 목표거리(row-hold)를 함께 유지하며
  로봇 왼쪽(정측면, 90도) LiDAR 섹터 하나만으로 "가깝다(장애물 옆)/멀다(빈 공간)"를
  판정한다. 이 판정을 BEFORE_BOX1 -> AT_BOX1 -> IN_GAP 순으로 추적하다가, IN_GAP
  상태에서 다시 "가깝다"(다음 장애물, box2)로 바뀌는 순간 갭의 진입/이탈 지점을
  기록하고 `RETURN_SCAN`으로 전환한다.
- `RETURN_SCAN` (반대 방향 재측정, **신설**): SEARCH의 측정은 항상 전진(+x) 중에만
  이뤄져서, 디바운스 지연이나 라이다 섹터가 모서리를 비스듬히 붙잡는 정도 같은
  "이동 방향에 따라 달라지는" 오차가 상쇄되지 않고 그대로 slot_center_x에 남는다
  (실측: world 중심 0.0m가 항상 +0.06~0.07m쪽으로 편향). box2를 지나친 지점에서
  살짝만 후진해 "이격"(box2에서 벗어남)을 반대 방향(-x)에서 다시 재는 것만으로
  충분하다 - box1까지 다시 갈 필요는 없다:
    gap_entry_pos = 실제 box1 경계 + 편향   (전진 중 "이격" 전이, +x 방향 지연)
    return_exit_pos = 실제 box2 경계 - 편향  (후진 중 "이격" 전이, -x 방향 지연)
  둘 다 같은 종류의 전이(근접->이격)라 편향 크기가 같고 부호만 반대이므로,
  slot_center = (gap_entry_pos + return_exit_pos) / 2 로 계산하면 편향 항이
  대수적으로 정확히 상쇄되어 box1/box2 각각의 실제 경계 평균이 그대로 나온다.
  재측정이 끝나면 `BACK_TO_CENTER`로 전환한다(실패 시 SEARCH의 편측 값으로 폴백).
- `BACK_TO_CENTER` (슬롯 중심 복귀): 기록해둔 슬롯 중심 x좌표까지 헤딩 + row-hold를
  유지한 채 이동해 되돌아온다 - 이 row-hold가 곧 "제자리 회전 전 위치 조정" 과정이다
  (헤딩만 잡으면 SEARCH부터 누적된 횡방향 드리프트가 그대로 남아 회전/후진이
  삐뚤어짐). box1/box2 감지는 차체 중심이 아니라 그보다 앞(LIDAR_OFFSET_X) 달린
  라이다 위치에서 일어나므로, 감지 시점의 차체 위치가 목표(slot_center_x)보다 이미
  넘어가 있을 수도(후진 필요), 못 미쳐 있을 수도(전진 필요) 있다 - 그래서 방향을
  후진으로 고정하지 않고 전환 시점에 계산한다(self.back_direction).
- `ROTATE_IN_PLACE` (제자리 회전): 반드시 장애물 사이로 들어가기 **전**, 아직 차선
  위(개활지)에 있을 때 끝내야 한다. 차체가 1.4m(길이) x 0.775m(폭)라 제자리 회전
  시 대각선 반경이 약 0.8m인데, 좌우 장애물 사이 빈 폭은 1.2m뿐이라 그 사이에 낀
  채로 돌면 반드시 스친다. 스폰 시 차선 y옵셋(-0.6m)을 충분히 둔 것도 이 회전
  반경을 위한 여유다. 목표 자세(-90도, 슬롯에 대해 수직)까지 돈다.
  실제로는 완전한 점 회전이 아니라 바퀴가 살짝 미끄러지며(스크럽) 회전 중 차체
  중심이 진행방향(+x)으로 밀리는 물리 현상이 있다 - 회전 종료 시점에 직접 정밀
  측정한 결과 회전 속도(0.25~0.6 rad/s)와 무관하게 약 +0.051m로 상당히 일관돼서,
  `ROTATE_SCRUB_DRIFT`로 BACK_TO_CENTER 목표를 미리 반대로 당겨 보정한다.
- `STRAIGHT_REVERSE` (직진 후진): 회전이 끝나면 헤딩(-90도) 유지만으로 직진
  후진한다. 정지는 후방 감지가 아니라 "차체 중심이 목표 정차 위치(슬롯 중앙,
  오도메트리 기준)에 도달했는가"로 판단한다. 회전 시작 위치 자체가 RETURN_SCAN +
  ROTATE_SCRUB_DRIFT 보정으로 이미 정확하므로, 후진 중 추가 정렬 없이도 최종
  좌우 간극이 거의 대칭으로 나온다(실측: 오차 약 1cm 수준).
- 최상위 긴급회피: 가장 가까운 장애물의 방위 반대쪽으로 단순 조향(회전+후퇴 없이
  전진 방향 유지 + 각속도만으로 회피). 충돌 일보 직전의 최후 방어선이므로 예측
  가능하고 빠른 단순 규칙을 쓴다. 단, 이 회피도 라이다가 보는 전방+좌우 250도
  범위 안에서만 동작한다(정후방 110도는 애초에 감지 대상이 아님).

장애물 배치 (worlds/parking_lot.sdf 기준):
  - 슬롯 좌/우 옆 칸 2개 : SEARCH/RETURN_SCAN의 편측 갭 탐지 대상(box1 -> 갭 -> box2).
  - 슬롯 자체(y=[0, 1.6])는 완전히 비어 있다. 보도블록 쪽에 있던 장애물은 슬롯
    경계(정지선, y=1.6) 밖 - 정지선에서 200mm 더 간 뒤에야 있으므로 정상 주차
    동작에는 관여하지 않는다.

자세(yaw)와 위치는 /odom(Gazebo DiffDrive 플러그인이 직접 발행하는 오도메트리)에서
받는다. odom 프레임은 스폰 시점(sim.launch.py의 -x -2.0 -y -0.6 -Y 0.0)을 원점으로
하므로, world/슬롯 좌표계로 변환할 때는 SPAWN_X/SPAWN_Y를 그대로 더하면 된다(스폰
요(yaw)가 0이라 회전 변환은 필요 없음). 위치는 목표 정차 위치 도달 판정(STRAIGHT_
REVERSE)과 "얼마나 진행했는지"를 재는 안전 폴백 용도로 쓰고, 그 외 상태 전이의
1차 판단 기준은 항상 LiDAR다.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# ==========================================================================
# 튜닝 상수 (실제 Gazebo에서 여러 번 구동/계측하며 조정함 - 최종 확인된 정밀도:
# 회전 종료 시 요(yaw) 오차 0도, 최종 좌우 중심 오차 약 1cm)
# ==========================================================================
SPEED_SEARCH = 0.30          # SEARCH(차선 직진) 전진 속도 (m/s)
SPEED_BACK_TO_CENTER = -0.20 # BACK_TO_CENTER 속도 크기(부호는 self.back_direction이
                             # 정함, abs()로 사용) - 슬롯 중심으로 복귀
SPEED_STRAIGHT = -0.15       # STRAIGHT_REVERSE 상태 후진 속도
SPEED_ESCAPE = 0.30          # 긴급 회피 시 전진 속도

ROTATE_SPEED = 0.6          # ROTATE_IN_PLACE 최대 각속도 (rad/s) - 0.25로 늦춰봤더니
                             # 회전 스크럽 드리프트(ROTATE_SCRUB_DRIFT 참고)가 오히려
                             # 더 커져서(+0.05m, 원래보다 나쁨) 되돌림 - 느리게 돈다고
                             # 슬립이 줄어들지 않고, 오히려 회전에 걸리는 총 시간이
                             # 길어질수록 누적 드리프트가 커지는 쪽으로 보인다.
K_ROTATE = 1.5               # 회전 목표각 오차 -> 각속도 비례 게인 (위와 동일 이유로 되돌림)
K_HEADING_HOLD = 1.0         # SEARCH/BACK_TO_CENTER/STRAIGHT_REVERSE 중 직진 유지용 헤딩 P게인

# --- 최상위 긴급회피 (가장 가까운 장애물 반대쪽으로 단순 회전) ---
# 라이다 FOV가 전방+좌우 250도뿐이라, 이 회피도 그 범위 안의 장애물에만 반응한다.
EMERGENCY_DIST = 0.12      # 전방위 어디서든 이 거리 안으로 들어오면 최우선 회피

# --- 좌측 정측면 섹터 : 라이다 FOV가 -125~+125도(250도)라 90도 정중앙을 써도
#     양쪽으로 15도 여유(105~120도까지)를 두고 완전히 FOV 안에 들어온다(예전
#     180도 FOV일 땐 90도가 경계값이라 75도로 물러나야 했음). SEARCH/RETURN_SCAN의
#     편측 갭 탐지, row-hold(횡방향 위치 유지) 모두 이 섹터를 공용으로 쓴다. ---
SIDE_SECTOR_HALF_WIDTH = math.radians(5.0)  # 좁을수록 정확 - 섹터가 넓으면(과거 15도)
                             # 장애물 모서리를 실제 경계를 지난 뒤에도(box1) 또는 도달
                             # 하기 전에도(box2) 비스듬한 각도로 계속 붙잡아, 갭 진입/이탈
                             # 감지가 진행방향 기준 각각 다르게(비대칭으로) 밀린다(실측:
                             # box1 이탈 +0.30m 지연, box2 진입 -0.21m 조기 감지 -> 평균
                             # slot_center_x가 진행방향으로 약 4.5cm 밀림). 5도로 좁히면
                             # 이 비스듬한 모서리 포착 여유가 tan(5)/tan(15)=0.32배로
                             # 줄어 편향도 비례해서 작아진다.
LEFT_SECTOR_CENTER = math.pi / 2.0          # 로봇 좌측 정측면(장애물 행이 있는 쪽) - 90도가
                                             # 아닌 다른 각을 중심으로 쓰면 갭 진입/이탈 감지
                                             # 위치가 기하학적으로 밀려서(전방으로 치우쳐서)
                                             # slot_center_x가 크게 틀어지는 버그가 생긴다
                                             # (실측으로 확인: 75도 중심 사용 시 실제 중심
                                             # 0.0m 대신 1.20m로 계산됨).
ROW_NEAR_DIST = 1.0        # 이 값(m)보다 가까우면 "장애물 옆", 멀면 "빈 공간"으로 판정
                            # (스폰 y옵셋 -0.6m 기준: 장애물 옆 ~0.75m, 갭 구간 ~1.15~2.2m
                            # 이므로 그 사이인 1.0m을 경계로 삼음 - 차선 y옵셋을 바꾸면
                            # 이 값도 같이 재계산해야 함)
ROW_DEBOUNCE_COUNT = 3      # 이 횟수(제어주기)만큼 연속으로 같은 판정이 나와야 확정
                            # (라이다 노이즈로 인한 순간적 오판정 필터링)

# --- SEARCH/BACK_TO_CENTER 중 옆 장애물과의 거리 유지(row-hold) : 헤딩만 잡으면
#     회전 전까지 몇 미터를 주행하는 동안 횡방향 드리프트가 누적되어 회전/후진이
#     삐뚤어지므로, 편측 라이다 거리를 목표값에 붙잡아 실제 위치도 함께 보정한다 ---
ROW_HOLD_DIST = 0.75       # 옆 장애물 옆을 지날 때 유지할 목표 수직 거리(m) -
                            # 스폰 y옵셋(-0.6m) 기준 장애물 근접 거리 실측값(~0.75m)
K_ROW_HOLD = 0.6            # 목표거리 오차 -> 각속도 보정 비례 게인
MAX_ROW_CORRECTION = 0.25   # 위 보정 각속도 상한(rad/s) - 헤딩 유지 항과 합산되므로 제한

TARGET_YAW = -math.pi / 2.0    # 슬롯에 대해 수직으로 선 상태 (회전 목표 자세)
ANGLE_TOL = math.radians(0.3)  # 예전엔 3도라 STRAIGHT_REVERSE 시작 시점에 잔여 각오차가
                             # 남아있었고, 그게 후진 중 옆으로 밀리는 변위(약 3.5cm)로
                             # 누적되는 원인 중 하나로 추정됨 - 훨씬 좁혀서 회전을 더
                             # 정밀하게 끝낸다.

# --- 슬롯/차체 규격 상수 (후진주차장.png, 차량규격.png 기준 - 센서로 재는 값이
#     아니라 이미 알려진 환경/설계 상수) ---
LIDAR_OFFSET_X = 0.65       # urdf의 lidar_offset_x와 반드시 일치시킬 것. SEARCH 중
                             # box1/box2 감지는 "차체 중심"이 아니라 "라이다"가 그
                             # 위치에 왔을 때 일어나므로, gap_entry/gap_exit에 기록되는
                             # self.x(차체 중심)는 실제 물리적 경계보다 항상 이만큼
                             # (라이다가 차체보다 앞서 있는 만큼) 덜 온 값이다. 이 보정을
                             # 빼먹으면 entry/exit 평균(slot_center_x)이 실제 갭 중심에서
                             # 이 오프셋만큼 그대로 밀려서 나온다 - 실측으로 확인됨
                             # (world 좌표 기준 참 중심 0.0m인데 보정 전엔 -0.65m로 계산됨).
CONTROL_PERIOD = 0.1        # 10 Hz
ROTATE_SCRUB_DRIFT = 0.051  # ROTATE_IN_PLACE 중 바퀴가 완전한 점 회전이 아니라 살짝
                             # 미끄러지며(스크럽) 회전축이 그대로 진행방향(+x)으로
                             # 밀리는 물리 현상. self.x-self.slot_center_x를 회전
                             # 종료 시점에 직접 로그로 정밀 측정한 결과 +0.0508,
                             # +0.0512로 회전 속도(0.25 vs 0.6 rad/s)와 무관하게
                             # 놀라울 정도로 일관됨(예전 대략치 0.04는 부정확한
                             # 측정이었음) - 좌우 정렬/디바운스 등 제어 로직과
                             # 무관하게 항상 발생. BACK_TO_CENTER 목표를 이만큼
                             # 반대로 당겨서 회전 후 실제 위치가 진짜 중앙에 오도록
                             # 미리 보정한다.
SPAWN_X = -2.0              # sim.launch.py 스폰 x (odom 프레임 원점의 world 좌표)
SPAWN_Y = -0.6              # sim.launch.py 스폰 y
SLOT_ENTRY_Y = 0.0          # 슬롯 입구 (world/슬롯 좌표계)
SLOT_BACK_BOUNDARY_Y = 1.6  # 슬롯 안쪽 경계(정지선) - 이 너머는 슬롯 밖
TARGET_CENTER_WORLD_Y = (SLOT_ENTRY_Y + SLOT_BACK_BOUNDARY_Y) / 2.0  # 0.8m
CENTER_Y_TOL = 0.03         # 목표 정차 위치 도달 허용 오차(m)

# --- 안전 폴백 (LiDAR/오도메트리가 예상과 다를 때를 대비한 최후 방어선, 1차 기준 아님) ---
MAX_SEARCH_TRAVEL = 6.0     # 이만큼 이동해도 갭(box1->빈공간->box2)을 다 못 찾으면
                             # SEARCH 중단(FAILED)
MAX_RETURN_SCAN_TRAVEL = 0.5  # RETURN_SCAN(box2에서 살짝 후진)이 이만큼 이동해도
                             # "이격"을 못 찾으면 SEARCH의 편측 측정값만으로 폴백
                             # (정상 상황에서는 디바운스 지연 정도인 몇 cm면 충분함)
MAX_BACK_TRAVEL = 2.0       # BACK_TO_CENTER 진입 후 이만큼 이동해도 슬롯 중심에
                             # 못 이르면 그 자리에서 회전으로 넘어감(안전 폴백)
MAX_REVERSE_TRAVEL = 1.6    # STRAIGHT_REVERSE 시작 후 이만큼 후진해도 목표 정차
                             # 위치(오도메트리 기준)에 못 이르면 강제 정지(FINISHED) -
                             # 오도메트리 드리프트 등에 대비한 최후 방어선


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clip(value, lo, hi):
    return max(lo, min(hi, value))


class ParkingController(Node):

    def __init__(self):
        super().__init__('parking_controller')

        self.state = 'SEARCH'
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.have_odom = False
        self.scan = None

        self.search_start_x = 0.0
        self.search_start_y = 0.0
        self.search_start_set = False
        self.search_heading = 0.0

        # 편측 갭 탐지 상태
        self.row_phase = 'BEFORE_BOX1'   # BEFORE_BOX1 -> AT_BOX1 -> IN_GAP
        self.row_near = False
        self.row_pending = None
        self.row_pending_count = 0
        self.gap_entry_pos = None
        self.gap_exit_pos = None
        self.slot_center_x = 0.0

        # RETURN_SCAN: box2에서 살짝 후진해 "이격"을 반대 방향에서 재측정하는 상태
        # (전진 중 측정과 평균 내면 디바운스/센서 각도 등 방향 의존적 편향이 상쇄됨)
        self.return_near = True
        self.return_pending = None
        self.return_pending_count = 0
        self.return_exit_pos = None
        self.return_start_x = 0.0
        self.return_start_y = 0.0

        self.back_start_x = 0.0
        self.back_start_y = 0.0
        self.back_direction = -1.0
        self.straight_start_x = 0.0
        self.straight_start_y = 0.0

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_cb, 10)

        self.timer = self.create_timer(CONTROL_PERIOD, self.control_step)
        self.get_logger().info('parking_controller(차동구동 AMR, 전방+좌우 라이다) 시작 - 상태: SEARCH')

    def odom_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.have_odom = True

    def scan_cb(self, msg: LaserScan):
        self.scan = msg

    # ---- LiDAR 기반 거리 계산 (섹터 기반, 상태전이 트리거) --------------

    def _sector_min_range(self, center_angle, half_width):
        scan = self.scan
        best = float('inf')
        for i, r in enumerate(scan.ranges):
            if math.isnan(r) or math.isinf(r):
                continue
            angle = scan.angle_min + i * scan.angle_increment
            diff = wrap_to_pi(angle - center_angle)
            if abs(diff) <= half_width:
                best = min(best, r)
        return best

    def compute_row_distance(self):
        # 로봇 좌표계 기준 좌측 전방-측면 섹터의 최소 거리 - 장애물 행이 있는 쪽.
        return self._sector_min_range(LEFT_SECTOR_CENTER, SIDE_SECTOR_HALF_WIDTH)

    def compute_nearest_obstacle(self):
        scan = self.scan
        best_r = float('inf')
        best_angle = 0.0
        for i, r in enumerate(scan.ranges):
            if math.isnan(r) or math.isinf(r):
                continue
            if r < best_r:
                best_r = r
                best_angle = scan.angle_min + i * scan.angle_increment
        return best_angle, best_r

    def compute_emergency(self):
        _, best_r = self.compute_nearest_obstacle()
        return best_r < EMERGENCY_DIST

    def _lane_hold_angular(self, target_heading):
        """헤딩을 target_heading으로 유지하면서, 옆(왼쪽) 라이다 거리가 ROW_HOLD_DIST
        보다 가까워졌을 때만(=장애물에 너무 붙었을 때만) 멀어지는 쪽으로 미세 조향을
        더한다. SEARCH/BACK_TO_CENTER 공용 - 회전 전 위치 조정 담당.

        편측(장애물 쪽으로 끌어당기지 않는) 보정인 이유: 목표거리로 끌어당기는
        양방향 보정을 썼더니, 실제로는 더 멀리 있는 게 안전한데도 목표거리 쪽으로
        끌어당겨져 slot_center_x 계산이 장애물 쪽으로 밀리고 회전 시 차체 폭이
        옆 장애물에 걸치는 문제가 생겼다(실측: 슬롯 중심 world x가 0.0 대신
        0.22로 계산되어 회전 중 우측 장애물에 살짝 겹침). 이미 충분히 멀면
        그대로 두고, 너무 가까워졌을 때만 멀어지게 해 항상 여유 쪽으로만
        보정한다."""
        heading_term = -K_HEADING_HOLD * wrap_to_pi(self.yaw - target_heading)
        row_dist = self.compute_row_distance()
        row_term = 0.0
        if not math.isinf(row_dist) and row_dist < ROW_HOLD_DIST:
            row_term = clip(K_ROW_HOLD * (row_dist - ROW_HOLD_DIST),
                             -MAX_ROW_CORRECTION, 0.0)
        return clip(heading_term + row_term, -0.5, 0.5)

    def world_y(self):
        # odom 프레임(스폰 지점 원점) -> 슬롯/world y좌표. 스폰 요(yaw)가 0이므로
        # 단순 평행이동만 하면 된다.
        return SPAWN_Y + self.y

    # ---- SEARCH: 편측 갭 탐지 (box1 -> 빈공간 -> box2) ----------------------

    def update_row_gap_detection(self):
        """매 제어주기 로봇 왼쪽(전방-측면) 섹터를 보고 BEFORE_BOX1 -> AT_BOX1 ->
        IN_GAP 순으로 진행시킨다. IN_GAP에서 다시 장애물이 잡히면(box2) 갭 중심을
        계산해 True를 반환(BACK_TO_CENTER로 전환하라는 신호), 그 전까지는 False."""
        raw_near = self.compute_row_distance() < ROW_NEAR_DIST

        if raw_near == self.row_near:
            self.row_pending_count = 0
            self.row_pending = None
            return False

        if self.row_pending != raw_near:
            self.row_pending = raw_near
            self.row_pending_count = 1
        else:
            self.row_pending_count += 1

        if self.row_pending_count < ROW_DEBOUNCE_COUNT:
            return False

        # 확정된 전이
        self.row_near = raw_near
        self.row_pending_count = 0

        if self.row_phase == 'BEFORE_BOX1' and self.row_near:
            self.row_phase = 'AT_BOX1'
            self.get_logger().info('SEARCH: 첫 번째 장애물(box1) 옆 통과 중')
        elif self.row_phase == 'AT_BOX1' and not self.row_near:
            self.row_phase = 'IN_GAP'
            self.gap_entry_pos = (self.x, self.y)
            self.get_logger().info(f'SEARCH: 갭 진입 감지 (x={self.x:.2f})')
        elif self.row_phase == 'IN_GAP' and self.row_near:
            self.gap_exit_pos = (self.x, self.y)
            self.get_logger().info(f'SEARCH: 갭 이탈 감지 (x={self.x:.2f}) - box2 도달')
            return True

        return False

    # ---- RETURN_SCAN: 후진하며 box2/box1 경계 반대 방향 재측정 ----------------

    def update_return_scan(self):
        """SEARCH가 box1을 "이탈"(근접->이격, +x 방향 이동 중)한 지점(gap_entry_pos)에는
        방향 의존적 편향(디바운스 지연, 라이다 섹터가 모서리를 비스듬히 붙잡는 정도)이
        그대로 남아있다. box2에서 살짝만 후진해 같은 종류의 전이(근접->이격, 이번엔
        -x 방향 이동 중)를 한 번 더 재면, 같은 종류의 전이는 방향만 반대이므로 편향도
        부호만 반대로 걸린다 - box1 쪽까지 다시 갈 필요 없이 이 두 값만 평균 내도
        슬롯 중심이 나온다(자세한 유도는 모듈 docstring 참고). box2에서 "이격"으로
        바뀌는 순간(단 한 번의 전이) True를 반환한다."""
        raw_near = self.compute_row_distance() < ROW_NEAR_DIST

        if raw_near == self.return_near:
            self.return_pending_count = 0
            self.return_pending = None
            return False

        if self.return_pending != raw_near:
            self.return_pending = raw_near
            self.return_pending_count = 1
        else:
            self.return_pending_count += 1

        if self.return_pending_count < ROW_DEBOUNCE_COUNT:
            return False

        self.return_near = raw_near
        self.return_pending_count = 0

        if not self.return_near:
            self.return_exit_pos = (self.x, self.y)
            self.get_logger().info(f'RETURN_SCAN: box2 이탈 재감지 (x={self.x:.2f})')
            return True

        return False

    # ---- 제어 루프 -------------------------------------------------------------

    def control_step(self):
        if not self.have_odom or self.scan is None:
            return

        linear = 0.0
        angular = 0.0

        # [최상위 알고리즘] FINISHED/FAILED 가 아닌 한 항상 최우선으로 검사.
        # 충돌 일보 직전의 최후 방어선이므로 가장 단순하고 예측 가능한 규칙을 쓴다:
        # 가장 가까운 장애물의 방위 반대쪽으로 회전하며 전진. (라이다가 못 보는
        # 후방 장애물은 이 회피의 대상이 될 수 없다 - 애초에 감지 불가능한 영역.)
        if self.compute_emergency() and self.state not in ('FINISHED', 'FAILED'):
            angle, _ = self.compute_nearest_obstacle()
            linear = SPEED_ESCAPE
            angular = -ROTATE_SPEED if 0.0 < wrap_to_pi(angle) <= math.pi else ROTATE_SPEED

        else:
            if self.state == 'SEARCH':
                if not self.search_start_set:
                    self.search_start_x, self.search_start_y = self.x, self.y
                    self.search_heading = self.yaw
                    self.search_start_set = True

                # 차선을 따라 직진 헤딩 유지(스폰 시의 헤딩을 기준으로 삼는다) +
                # 옆 장애물과의 거리 유지(row-hold)로 횡방향 드리프트 보정
                linear = SPEED_SEARCH
                angular = self._lane_hold_angular(self.search_heading)

                if self.update_row_gap_detection():
                    self.state = 'RETURN_SCAN'
                    self.return_start_x, self.return_start_y = self.x, self.y
                    self.get_logger().info(f'SEARCH -> RETURN_SCAN (x={self.x:.2f})')
                elif math.hypot(self.x - self.search_start_x,
                                 self.y - self.search_start_y) > MAX_SEARCH_TRAVEL:
                    self.state = 'FAILED'
                    self.get_logger().error('SEARCH 실패: 빈자리를 찾지 못하고 MAX_SEARCH_TRAVEL 초과')

            elif self.state == 'RETURN_SCAN':
                # box2를 막 지나친 상태에서 살짝만 후진해 "box2에서 벗어남"을
                # 반대 방향(-x)에서 다시 재는, 아주 짧은 재측정이다(모듈 docstring/
                # update_return_scan 참고 - box1까지 다시 갈 필요 없음).
                linear = -abs(SPEED_BACK_TO_CENTER)
                angular = self._lane_hold_angular(self.search_heading)

                if self.update_return_scan():
                    # true_center = (gap_entry_pos + return_exit_pos) / 2 - 두 값 모두
                    # "근접->이격" 전이(하나는 +x, 하나는 -x 방향)라 방향 의존적 편향이
                    # 부호만 반대로 걸려서 평균에서 상쇄된다(유도는 모듈 docstring 참고).
                    self.slot_center_x = (
                        (self.gap_entry_pos[0] + self.return_exit_pos[0]) / 2.0
                        + LIDAR_OFFSET_X - ROTATE_SCRUB_DRIFT
                    )
                    self.state = 'BACK_TO_CENTER'
                    self.back_start_x, self.back_start_y = self.x, self.y
                    # 방향에 따른 편향은 상쇄됐지만, LIDAR_OFFSET_X 보정 때문에
                    # 목표(slot_center_x)가 현재 위치보다 앞(+x)일 수도 뒤(-x)일
                    # 수도 있으므로 방향을 가정하지 않고 매번 계산한다.
                    self.back_direction = (
                        1.0 if (self.slot_center_x - self.x) * math.cos(self.search_heading) >= 0.0
                        else -1.0
                    )
                    self.get_logger().info(
                        f'RETURN_SCAN -> BACK_TO_CENTER (슬롯 중심 x={self.slot_center_x:.2f}, '
                        f'방향={"+" if self.back_direction > 0 else "-"})'
                    )
                elif math.hypot(self.x - self.return_start_x,
                                 self.y - self.return_start_y) > MAX_RETURN_SCAN_TRAVEL:
                    # 재측정 실패 - SEARCH의 편측(전진) 측정값만으로 폴백
                    self.slot_center_x = (
                        (self.gap_entry_pos[0] + self.gap_exit_pos[0]) / 2.0
                        + LIDAR_OFFSET_X - ROTATE_SCRUB_DRIFT
                    )
                    self.state = 'BACK_TO_CENTER'
                    self.back_start_x, self.back_start_y = self.x, self.y
                    self.back_direction = (
                        1.0 if (self.slot_center_x - self.x) * math.cos(self.search_heading) >= 0.0
                        else -1.0
                    )
                    self.get_logger().warn(
                        f'RETURN_SCAN -> BACK_TO_CENTER (box2 재이탈 감지 실패, 편측 측정값 폴백, '
                        f'슬롯 중심 x={self.slot_center_x:.2f})'
                    )

            elif self.state == 'BACK_TO_CENTER':
                # 슬롯 중심 x까지 헤딩 유지한 채 이동(방향은 SEARCH -> BACK_TO_CENTER
                # 전환 시점에 계산해둔 self.back_direction)
                linear = self.back_direction * abs(SPEED_BACK_TO_CENTER)
                angular = self._lane_hold_angular(self.search_heading)

                traveled = math.hypot(self.x - self.back_start_x, self.y - self.back_start_y)
                reached_center = (
                    (self.x - self.slot_center_x) * math.cos(self.search_heading)
                    * self.back_direction >= 0.0
                )

                if reached_center:
                    self.state = 'ROTATE_IN_PLACE'
                    self.get_logger().info(
                        f'BACK_TO_CENTER -> ROTATE_IN_PLACE (x={self.x:.2f}, 목표 {self.slot_center_x:.2f})'
                    )
                elif traveled > MAX_BACK_TRAVEL:
                    self.state = 'ROTATE_IN_PLACE'
                    self.get_logger().warn(
                        'BACK_TO_CENTER -> ROTATE_IN_PLACE (슬롯 중심 미도달, 안전거리 폴백으로 전환)'
                    )

            elif self.state == 'ROTATE_IN_PLACE':
                # 위치는 그대로, 목표 자세(슬롯에 대해 수직)까지 제자리 회전만 한다.
                # 반드시 장애물 사이로 들어가기 전(개활지)에 끝나야 한다 - 모듈
                # docstring의 ROTATE_IN_PLACE 설명 참고.
                linear = 0.0
                angle_error = wrap_to_pi(TARGET_YAW - self.yaw)
                angular = clip(K_ROTATE * angle_error, -ROTATE_SPEED, ROTATE_SPEED)
                if abs(angle_error) < ANGLE_TOL:
                    self.state = 'STRAIGHT_REVERSE'
                    self.straight_start_x, self.straight_start_y = self.x, self.y
                    self.get_logger().info(
                        f'ROTATE_IN_PLACE -> STRAIGHT_REVERSE (x={self.x:.4f}, '
                        f'스크럽 드리프트={self.x - self.slot_center_x:+.4f})'
                    )

            elif self.state == 'STRAIGHT_REVERSE':
                # 직진 후진하며 헤딩(-90도)만 유지한다. 후방은 못 보므로 정지는
                # 오도메트리 기준 목표 정차 위치 도달 여부로 판단한다.
                #
                # 좌우 라이다 거리차 기반 중앙 정렬을 추가해봤지만(각속도에 직접
                # 더하는 방식, 이후 목표각 오프셋으로 변환하는 방식 둘 다) 오히려
                # 결과가 나빠져서 뺐다: 좌/우 라이다가 후진 시작 시점부터 동시에
                # 잡히지 않고(한쪽은 초반부터, 반대쪽은 도착 직전에야 잡힘) 대부분의
                # 구간에서 보정이 꺼져있다가 막판에만 급하게 작동해 오히려 흔들렸다.
                # 대신 BACK_TO_CENTER/RETURN_SCAN에서 회전 시작 위치 자체를 정확히
                # 맞추는 쪽으로 해결했다 - 위치가 정확하면 순수 헤딩 유지만으로
                # 충분하다(모듈 docstring 참고).
                linear = SPEED_STRAIGHT
                angular = clip(-K_HEADING_HOLD * wrap_to_pi(self.yaw - TARGET_YAW), -0.5, 0.5)

                traveled = math.hypot(self.x - self.straight_start_x,
                                       self.y - self.straight_start_y)
                reached_target = self.world_y() >= (TARGET_CENTER_WORLD_Y - CENTER_Y_TOL)

                if reached_target:
                    self.state = 'FINISHED'
                    self.get_logger().info(
                        f'STRAIGHT_REVERSE -> FINISHED (목표 정차 위치 도달, world_y={self.world_y():.2f})'
                    )
                elif traveled > MAX_REVERSE_TRAVEL:
                    self.state = 'FINISHED'
                    self.get_logger().warn(
                        'STRAIGHT_REVERSE -> FINISHED (목표 정차 위치 미도달, 안전거리 폴백으로 정지)'
                    )

            else:  # FAILED, FINISHED
                linear, angular = 0.0, 0.0

        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ParkingController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
