#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 테스크 매니저 (Python) — 동작 가능한 베이스라인 (+ 자동 드롭오프 범위 모드)
==========================================================================

여러 대의 Nav2 로봇(TurtleBot3 등)을 중앙에서 제어하기 위한 참고 구현.

핵심 기능 요약
--------------
- NavigateToPose 액션으로 목표 전송 (로봇 네임스페이스별)
- 작업 큐(Deque) + 단순 할당 정책(Idle 우선, ETA 근사)
- 진행 모니터링(정체/타임아웃 시 취소 및 재할당)
- 충돌/헤드온 완화 휴리스틱(동일 ETA·근접 시 보류)
- 드롭오프 자동 태스크 생성(point/circle/rect)

실행 전제
---------
1) ROS 2 Humble + Nav2 설치 및 bringup 실행
   - 로봇별 네임스페이스 예: tb1, tb3
   - 액션 서버: /<ns>/navigate_to_pose
   - 위치 추정: /<ns>/amcl_pose (PoseWithCovarianceStamped)

2) 이 스크립트를 포함하는 패키지(예: fleet_manager)로 설치 후 실행
   - 예: ros2 run <패키지명> task_manager
   - 또는 python 직접 실행 시 PYTHONPATH 환경 설정 필요

3) 파라미터 주요 키(ros2 param / launch로 주입)
   - robot_namespaces       : ["tb1","tb3"] 처럼 배열
   - nominal_speed_mps      : ETA 근사에 쓰는 명목 속도(m/s)
   - stuck_timeout_sec      : 피드백 정체 감지 기준(초)
   - goal_timeout_sec       : 목표 도달 타임아웃(초)
   - coordination_radius_m  : 근접 충돌 휴리스틱 반경(미터)
   - required_sites_yaml    : 방문해야 할 위치 목록(YAML 파일 경로)
   - dry_run                : True면 Nav2에 실제 Goal 미전송(로직만)

4) 자동 드롭오프(배송 종료 후 반납/정렬 지점으로 유도)
   - auto_deliver_enabled : Bool 토글(서비스 /fleet/auto_deliver 로도 변경 가능)
   - dropoff_mode         : "point" | "circle" | "rect"
   - 모드별 파라미터는 아래 "자동 드롭오프 파라미터" 참고

입력 YAML 예시(required_sites_yaml)
-----------------------------------
- id: A1
  x: 1.0
  y: 2.0
  yaw_deg: 0.0
  priority: 10
- id: B2
  x: -3.5
  y: 0.8
  yaw_deg: 90.0
  priority: 5

자동 드롭오프 파라미터
---------------------
- point:
  dropoff_x, dropoff_y, dropoff_yaw_deg
- circle:
  dropoff_center_x, dropoff_center_y, dropoff_radius_m
  (로봇이 원 밖이면 원 경계 최근접점, 원 안·중심이면 중심으로)
- rect:
  dropoff_rect_cx, dropoff_rect_cy, dropoff_rect_yaw_deg,
  dropoff_rect_w, dropoff_rect_h
  (회전된 사각형 내부로 최근접점 클램프 → 그 점으로 유도)

드라이런(dry_run=True)
----------------------
- Nav2 액션 서버 연결 없이 큐·할당·보류·요약 등 로직만 검증.

확장 포인트(수업/대회용 TODO)
-----------------------------
- Allocation.pick(): 우선순위/데드라인/거리/배터리/공정성 등 고려
- sequencing_baseline(): TSP 근사, 클러스터링, 지역-전역 혼합 등
- _will_conflict(): 교차/통로폭/정지선/원활한 교행을 위한 규칙 강화
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Deque, List, Optional, Tuple
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_srvs.srv import SetBool


# ------------------------------
# 데이터 모델
# ------------------------------
@dataclass(order=True)
class Task:
    """
    '하나의 목표 지점'을 나타내는 작업 단위.
    - 우선순위(priority)가 클수록 먼저 처리.
    - deadline_sec 등 스케줄링 힌트 제공 가능.
    - is_delivery: 자동 드롭오프에서 생성한 태스크인지 표시.
    - preferred_robot: 특정 로봇이 처리해야 하는 경우 지정.
    """
    # 우선순위 큐 정렬 키(내부용): (-priority, created_ts)
    sort_index: float = field(init=False, repr=False)

    # 태스크 고유 식별자 및 목표 포즈(map 기준)
    id: str
    x: float
    y: float
    yaw_rad: float = 0.0

    # 스케줄링 힌트(선택)
    priority: int = 0
    deadline_sec: Optional[float] = None
    created_ts: float = field(default_factory=lambda: time.time())

    # 드롭오프 연쇄 제어
    is_delivery: bool = False
    preferred_robot: Optional[str] = None

    def __post_init__(self):
        # 우선순위 큐(힙)에서 "priority 큰 것 우선"을 위해 -priority 사용
        self.sort_index = (-self.priority, self.created_ts)

    def to_pose_stamped(self, frame_id: str = "map") -> PoseStamped:
        """
        Nav2 액션에 전송할 PoseStamped 메세지로 변환.
        - 헤더 시간은 현재 Clock().now() 사용
        - yaw→Quaternion 변환은 z축 회전(roll=pitch=0 가정)
        """
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = Clock().now().to_msg()
        ps.pose.position = Point(x=self.x, y=self.y, z=0.0)
        qx, qy, qz, qw = yaw_to_quat(self.yaw_rad)
        ps.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        return ps


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    """
    yaw(라디안) → 쿼터니언(x,y,z,w). roll=pitch=0 가정.
    - z축 회전만 고려하는 단순화.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (0.0, 0.0, sy, cy)


@dataclass
class RobotState:
    """
    로봇의 현재 상태(최근 포즈/목표/진행도/실패횟수).
    - busy: 액션 진행 중 여부
    - last_progress_ts: 피드백(잔여거리 감소 등) 마지막 갱신 시각
    - failed_count: 연속 실패 횟수(정책에 활용 가능)
    """
    ns: str
    pose: Optional[PoseStamped] = None
    goal: Optional[PoseStamped] = None
    last_progress_ts: float = field(default_factory=lambda: time.time())
    busy: bool = False
    failed_count: int = 0


# ------------------------------
# 로봇 에이전트(로봇별 래퍼)
# ------------------------------
class RobotAgent(Node):
    """
    개별 로봇을 감싸는 래퍼 노드.
    - /<ns>/amcl_pose 구독으로 자기 위치 추적
    - /<ns>/navigate_to_pose 액션 클라이언트로 목표 전송
    - 누적 주행거리 측정(odometer) 지원
    """
    def __init__(self, ns: str):
        # 노드 이름은 고유, 네임스페이스는 실제 토픽/액션 prefix에 반영
        super().__init__(f"robot_agent_{ns}", namespace=ns)

        self.ns = ns
        self.state = RobotState(ns=ns)

        # QoS: 위치 추정은 신뢰성(RELIABLE), 최근 10개 버퍼
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        # /<ns>/amcl_pose 구독 (PoseWithCovarianceStamped)
        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            f"/{ns}/amcl_pose",
            self._on_amcl_pose,
            qos
        )

        # Nav2 NavigateToPose 액션 클라이언트(네임스페이스 포함)
        self._nav_client = ActionClient(self, NavigateToPose, f"/{ns}/navigate_to_pose")

        # 내부 액션 핸들 및 진행계측
        self._goal_handle = None
        self._send_time = 0.0
        self._last_xy: Optional[Tuple[float, float]] = None
        self._dist_m: float = 0.0  # 누적 주행거리(m)

        self.get_logger().info(f"RobotAgent for '{ns}' ready → action: /{ns}/navigate_to_pose")

    def reset_odometer(self):
        """요약 통계를 위해 주행거리 적산기 초기화."""
        self._dist_m = 0.0
        if self.state.pose is not None:
            p = self.state.pose.pose.position
            self._last_xy = (p.x, p.y)
        else:
            self._last_xy = None

    @property
    def total_distance_m(self) -> float:
        """현재 배치 동안의 누적 주행거리(m)."""
        return self._dist_m

    # ── 콜백: AMCL 포즈 수신 시 상태 갱신 + 주행거리 적산
    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose

        x, y = ps.pose.position.x, ps.pose.position.y
        if self._last_xy is not None:
            dx = x - self._last_xy[0]
            dy = y - self._last_xy[1]
            self._dist_m += math.hypot(dx, dy)
        self._last_xy = (x, y)

        self.state.pose = ps

    # ── 동기화가 쉬운 async API(직접 사용은 현재 경로 밖)
    async def go_to(self, goal: PoseStamped) -> bool:
        """
        단독 사용용(예: 테스트): go_to → wait_result 조합.
        본 파일에서는 TaskManager가 send_goal_async 콜백 체인으로 운용.
        """
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("NavigateToPose action server not available")
            return False

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal

        self._send_time = time.time()
        self.state.goal = goal
        self.state.busy = True

        self.get_logger().info(
            f"[{self.ns}] Sending goal → x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}"
        )

        send_goal_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._feedback_cb  # 잔여거리 등 피드백 반영
        )
        goal_handle = await send_goal_future
        if not goal_handle.accepted:
            self.get_logger().warn(f"[{self.ns}] Goal rejected by server")
            self.state.busy = False
            return False

        self._goal_handle = goal_handle
        return True

    async def wait_result(self) -> Optional[int]:
        """go_to와 세트: 결과 코드(GoalStatus.*) 반환."""
        if self._goal_handle is None:
            return None
        result_future = self._goal_handle.get_result_async()
        result = await result_future
        return int(result.status)

    def cancel(self):
        """현재 목표 취소 요청(모니터링에서 타임아웃/정체 시 호출)."""
        if self._goal_handle is not None:
            self.get_logger().warn(f"[{self.ns}] Canceling current goal…")
            self._goal_handle.cancel_goal_async()

    def _feedback_cb(self, feedback: NavigateToPose.Feedback):  # type: ignore[attr-defined]
        """
        Nav2 피드백 콜백.
        - distance_remaining 등 수신 시 '진행 중'으로 판단해 last_progress_ts 갱신.
        - 피드백 타입은 미들웨어 구현에 따라 래핑될 수 있어 예외 보호.
        """
        try:
            dist = feedback.feedback.distance_remaining  # type: ignore[attr-defined]
        except Exception:
            dist = None
        if dist is not None:
            self.state.last_progress_ts = time.time()


# ------------------------------
# 작업 할당 전략 (베이스라인: 단순 idle-first)
# ------------------------------
class Allocation:
    """
    아주 단순한 할당기.
    - idle이면서 포즈를 아는 로봇을 우선 선택
    - 모두 포즈 미상 → idle인 로봇 중 임의 선택
    향후 확장: ETA, 우선순위, 공정성, 배터리, 로드밸런싱 등.
    """
    def __init__(self, nominal_speed: float = 0.25):
        # ETA 근사에 사용할 명목 속도(정책 파라미터)
        self.nominal_speed = max(1e-3, nominal_speed)

    def pick(self, robots: List[RobotAgent], task: Task) -> Optional[RobotAgent]:
        # 1) idle + pose known
        for r in robots:
            if not r.state.busy and r.state.pose is not None:
                return r
        # 2) idle
        for r in robots:
            if not r.state.busy:
                return r
        return None


# ------------------------------
# 매니저 노드(중앙 조정자)
# ------------------------------
class TaskManager(Node):
    """
    중앙 테스크 매니저.
    - 작업 큐에서 태스크를 꺼내 로봇에 전송
    - 결과/피드백 모니터링 및 재큐잉
    - 자동 드롭오프 태스크 생성
    - 간단한 헤드온 완화 휴리스틱
    """
    def __init__(self):
        super().__init__("multi_robot_controller")

        # ── 1) 파라미터 선언(기본값 포함)
        self.declare_parameter("robot_namespaces", ["tb1", "tb3"])
        self.declare_parameter("nominal_speed_mps", 0.25)
        self.declare_parameter("stuck_timeout_sec", 20.0)  # 피드백 정체 감지
        self.declare_parameter("goal_timeout_sec", 180.0)  # 전체 목표 시간 초과
        self.declare_parameter("coordination_radius_m", 0.8)  # 근접 보류 반경
        self.declare_parameter("required_sites_yaml", "")
        self.declare_parameter("dry_run", False)

        # 자동 드롭오프(토글 + 모드/파라미터)
        self.declare_parameter("auto_deliver_enabled", False)
        self.declare_parameter("dropoff_mode", "rect")  # point | circle | rect

        # point 모드
        self.declare_parameter("dropoff_x", 0.0)
        self.declare_parameter("dropoff_y", 0.0)
        self.declare_parameter("dropoff_yaw_deg", 0.0)

        # circle 모드
        self.declare_parameter("dropoff_center_x", 0.0)
        self.declare_parameter("dropoff_center_y", 0.0)
        self.declare_parameter("dropoff_radius_m", 1.0)

        # rect 모드(회전된 사각형)
        self.declare_parameter("dropoff_rect_cx", 0.0)
        self.declare_parameter("dropoff_rect_cy", 0.0)
        self.declare_parameter("dropoff_rect_yaw_deg", 0.0)
        self.declare_parameter("dropoff_rect_w", 2.0)  # 가로(긴 변)
        self.declare_parameter("dropoff_rect_h", 1.0)  # 세로(짧은 변)

        # ── 2) 파라미터 로드
        ns_list = self.get_parameter("robot_namespaces").get_parameter_value().string_array_value
        nominal_speed = self.get_parameter("nominal_speed_mps").get_parameter_value().double_value
        self.stuck_timeout = self.get_parameter("stuck_timeout_sec").get_parameter_value().double_value
        self.goal_timeout = self.get_parameter("goal_timeout_sec").get_parameter_value().double_value
        self.coord_radius = self.get_parameter("coordination_radius_m").get_parameter_value().double_value
        self.dry_run = self.get_parameter("dry_run").get_parameter_value().bool_value

        self.auto_deliver = self.get_parameter("auto_deliver_enabled").get_parameter_value().bool_value
        self.drop_mode = self.get_parameter("dropoff_mode").get_parameter_value().string_value

        # point
        self.dropoff_x = self.get_parameter("dropoff_x").get_parameter_value().double_value
        self.dropoff_y = self.get_parameter("dropoff_y").get_parameter_value().double_value
        self.dropoff_yaw = math.radians(
            self.get_parameter("dropoff_yaw_deg").get_parameter_value().double_value
        )

        # circle
        self.drop_cx = self.get_parameter("dropoff_center_x").get_parameter_value().double_value
        self.drop_cy = self.get_parameter("dropoff_center_y").get_parameter_value().double_value
        self.drop_r = max(0.05, self.get_parameter("dropoff_radius_m").get_parameter_value().double_value)

        # rect
        self.drop_rx = self.get_parameter("dropoff_rect_cx").get_parameter_value().double_value
        self.drop_ry = self.get_parameter("dropoff_rect_cy").get_parameter_value().double_value
        self.drop_r_yaw = math.radians(
            self.get_parameter("dropoff_rect_yaw_deg").get_parameter_value().double_value
        )
        self.drop_rw = max(0.05, self.get_parameter("dropoff_rect_w").get_parameter_value().double_value)
        self.drop_rh = max(0.05, self.get_parameter("dropoff_rect_h").get_parameter_value().double_value)

        # ── 3) 자동 드롭오프 토글 서비스(/fleet/auto_deliver)
        self.auto_srv = self.create_service(SetBool, "/fleet/auto_deliver", self._srv_set_auto_deliver)

        # ── 4) 할당기 준비
        self.allocator = Allocation(nominal_speed=nominal_speed)

        # ── 5) 작업 큐(선입선출 + 우선순위 고려는 sequencing에서)
        self.task_queue: Deque[Task] = deque()

        # ── 6) 로봇 에이전트 생성(네임스페이스별)
        self.robots: List[RobotAgent] = []
        for ns in ns_list:
            self.robots.append(RobotAgent(ns))

        # ── 7) 런타임 중 태스크 끼워넣기(/fleet/new_task)
        qos = QoSProfile(depth=10)
        self.new_task_sub = self.create_subscription(
            PoseStamped,
            "/fleet/new_task",
            self._on_new_task,
            qos
        )

        # ── 8) 주기 타이머: 디스패치/모니터 루프(500ms)
        self.dispatch_timer = self.create_timer(0.5, self._dispatch_loop)
        self.monitor_timer = self.create_timer(0.5, self._monitor_loop)

        # ── 9) 런 상태/통계
        self._run_started: bool = False
        self._t0: float = 0.0
        self._summary_done: bool = False

        # ── 10) 필수 지점 로드 → 시퀀싱 → 큐 적재
        sites_path = self.get_parameter("required_sites_yaml").get_parameter_value().string_value
        if sites_path:
            sites = self._load_required_sites(sites_path)
            ordered_tasks = sequencing_baseline(sites, self.robots)
            for t in ordered_tasks:
                self.task_queue.append(t)
            self.get_logger().info(f"필수 지점 {len(ordered_tasks)}건 큐 적재 완료")

        self.get_logger().info(
            f"robot controller up with robots: {', '.join(ns_list)} | dry_run={self.dry_run} | "
            f"auto_deliver={self.auto_deliver} drop_mode={self.drop_mode}"
        )

    # ------------------ 서비스/유틸 ------------------
    def _srv_set_auto_deliver(self, request: SetBool.Request, response: SetBool.Response):
        """
        /fleet/auto_deliver: 자동 드롭오프 On/Off 토글.
        - ros2 service call /fleet/auto_deliver std_srvs/srv/SetBool "{data: true}"
        """
        self.auto_deliver = bool(request.data)
        response.success = True
        response.message = f"auto_deliver set to {self.auto_deliver}"
        self.get_logger().warn(response.message)
        return response

    def _start_run(self):
        """배치 시작 시 통계 초기화."""
        if self._run_started:
            return
        self._run_started = True
        self._summary_done = False
        self._t0 = time.time()
        for r in self.robots:
            r.reset_odometer()
        self.get_logger().info("Run started: odometers reset and timer started.")

    def _all_done(self) -> bool:
        """큐가 비고 모든 로봇이 idle이면 배치 종료."""
        return (not self.task_queue) and all(not r.state.busy for r in self.robots)

    def _finalize_run(self):
        """배치 요약(경과 시간, 로봇별/총 주행거리)."""
        if self._summary_done:
            return
        elapsed = time.time() - self._t0
        per_robot = [(r.ns, r.total_distance_m) for r in self.robots]
        total_dist = sum(d for _, d in per_robot)

        self.get_logger().info("=== Controller Summary (current batch) ===")
        self.get_logger().info(f"Elapsed: {elapsed:.2f} s")
        for ns, d in per_robot:
            self.get_logger().info(f"  {ns}: {d:.3f} m")
        self.get_logger().info(f"Total distance: {total_dist:.3f} m")
        self.get_logger().info("====================================")

        self._summary_done = True
        self._run_started = False
        self._t0 = 0.0

    # ------------------ 입력/로딩 ------------------
    def _on_new_task(self, msg: PoseStamped):
        """
        /fleet/new_task 로 PoseStamped가 들어오면 즉시 큐에 삽입.
        - yaw는 orientation.z,w 만으로 계산(roll/pitch 0 가정)
        """
        yaw = quat_to_yaw(msg.pose.orientation)
        t = Task(id=f"task_{int(time.time()*1000)}", x=msg.pose.position.x, y=msg.pose.position.y, yaw_rad=yaw)
        self.task_queue.append(t)
        self.get_logger().info(f"Enqueued new task {t.id} at ({t.x:.2f},{t.y:.2f})")

    def _load_required_sites(self, path: str) -> List[Task]:
        """
        YAML 파일에서 필수 지점 목록 로드.
        - 각 항목: id,x,y,yaw_deg,priority,deadline_sec
        """
        if not path:
            return []
        try:
            import yaml
            with open(path, "r") as f:
                data = yaml.safe_load(f) or []
            tasks: List[Task] = []
            for item in data:
                yaw_deg = float(item.get("yaw_deg", 0.0))
                tasks.append(Task(
                    id=str(item["id"]),
                    x=float(item["x"]),
                    y=float(item["y"]),
                    yaw_rad=math.radians(yaw_deg),
                    priority=int(item.get("priority", 0)),
                    deadline_sec=float(item.get("deadline_sec")) if item.get("deadline_sec") is not None else None,
                ))
            return tasks
        except Exception as e:
            self.get_logger().error(f"Failed to load required sites: {e}")
            return []

    # ------------------ 디스패처(큐→로봇) ------------------
    def _dispatch_loop(self):
        """
        0.5초마다 실행. 큐 맨 앞 태스크를 꺼내 보낼 로봇을 선정.
        - preferred_robot가 있으면 해당 로봇 idle일 때만 전송
        - _will_conflict()가 True면 잠시 보류(다음 주기 재시도)
        - dry_run이면 실제 액션 전송 없이 즉시 성공 처리
        - 성공/실패/거절은 콜백 체인에서 후속 처리
        """
        if not self.task_queue:
            return
        task = self.task_queue[0]

        # 1) 선호 로봇 지정 태스크: idle일 때만 집행
        if task.preferred_robot is not None:
            for r in self.robots:
                if r.ns == task.preferred_robot and not r.state.busy:
                    if self._will_conflict(r, task):
                        return  # 잠깐 보류
                    self.task_queue.popleft()
                    goal = task.to_pose_stamped()
                    self._send_goal_via_callbacks(r, task, goal)
                    return
            # 선호 로봇이 바쁘면 이번 주기는 대기
            return

        # 2) 일반 태스크: 할당기에게 선택 위임
        rob = self.allocator.pick(self.robots, task)
        if rob is None:
            return

        # 3) 간단한 헤드온·근접 충돌 휴리스틱
        if self._will_conflict(rob, task):
            return  # 잠깐 보류

        # 4) 집행
        self.task_queue.popleft()
        goal = task.to_pose_stamped()
        self._send_goal_via_callbacks(rob, task, goal)

    def _send_goal_via_callbacks(self, rob: RobotAgent, task: Task, goal: PoseStamped):
        """
        send_goal_async → (응답) → get_result_async 콜백 체인 구성.
        - dry_run: 실제 전송 없이 성공 처리(로직 검증용)
        - 서버 미가용/거절: 태스크를 다시 큐에 삽입
        """
        if not self._run_started:
            self._start_run()

        # ── 드라이런: 액션 전송 없이 성공 처리
        if self.dry_run:
            self.get_logger().info(
                f"[{rob.ns}] (DRY-RUN) would send goal → x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}"
            )
            rob.state.goal = goal
            rob.state.busy = False
            self.get_logger().info(f"[{rob.ns}] (DRY-RUN) Task {task.id} SUCCEEDED")
            # 바로 다음 태스크도 집행 시도(파이프라인 유지)
            self._dispatch_loop()
            return

        # ── 액션 서버 대기(짧은 타임아웃)
        if not rob._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"[{rob.ns}] NavigateToPose action server not available")
            rob.state.failed_count += 1
            self.task_queue.append(task)  # 재시도 위해 다시 큐에
            self._dispatch_loop()
            return

        # ── 전송
        rob.state.goal = goal
        rob.state.busy = True
        rob._send_time = time.time()
        self.get_logger().info(
            f"[{rob.ns}] Sending goal → x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}"
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal

        send_future = rob._nav_client.send_goal_async(goal_msg, feedback_callback=rob._feedback_cb)

        # ── 응답 콜백(수락/거절)
        def _goal_response_cb(fut):
            try:
                goal_handle = fut.result()
            except Exception as e:
                # 네트워크·미들웨어 예외 보호
                self.get_logger().error(f"[{rob.ns}] send_goal_async failed: {e}")
                rob.state.busy = False
                self.task_queue.append(task)
                self._dispatch_loop()
                return

            if not goal_handle.accepted:
                self.get_logger().warn(f"[{rob.ns}] Goal rejected by server")
                rob.state.busy = False
                self.task_queue.append(task)
                self._dispatch_loop()
                return

            rob._goal_handle = goal_handle
            res_future = goal_handle.get_result_async()
            res_future.add_done_callback(lambda rf: self._on_result_cb(rf, rob, task))

        send_future.add_done_callback(_goal_response_cb)

    def _on_result_cb(self, result_future, rob: RobotAgent, task: Task):
        """
        결과 콜백: SUCCEEDED/ABORTED/CANCELED에 따라 후처리.
        - 성공: 자동 드롭오프 태스크(선호 로봇 지정, 높은 priority) 앞에 삽입
        - 실패/취소: 재큐잉(앞쪽에 넣어 빠른 재시도)
        - 모든 태스크 완료 시 배치 요약 출력
        """
        try:
            result = result_future.result()
            code = int(result.status)
        except Exception as e:
            self.get_logger().error(f"[{rob.ns}] get_result_async error: {e}")
            code = GoalStatus.STATUS_ABORTED

        rob.state.busy = False
        if code == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"[{rob.ns}] Task {task.id} SUCCEEDED")
            rob.state.failed_count = 0

            # 자동 드롭오프: 배송 태스크가 아니면 1회 생성
            if self.auto_deliver and (not task.is_delivery):
                deliver_task = self._compute_dropoff_task(rob, base_priority=task.priority)
                if deliver_task is not None:
                    # 즉시 처리되도록 큐 앞에 삽입
                    self.task_queue.appendleft(deliver_task)
                    self.get_logger().info(
                        f"[{rob.ns}] Auto-delivery enqueued → ({deliver_task.x:.2f},{deliver_task.y:.2f}) mode={self.drop_mode}"
                    )

        elif code == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f"[{rob.ns}] Task {task.id} CANCELED; requeuing…")
            rob.state.failed_count += 1
            self.task_queue.append(task)
        else:  # ABORTED 등
            self.get_logger().warn(f"[{rob.ns}] Task {task.id} ABORTED(code={code}); requeuing…")
            rob.state.failed_count += 1
            self.task_queue.append(task)

        # 다음 태스크 집행 시도
        self._dispatch_loop()
        # 배치 종료 검사
        if self._run_started and self._all_done():
            self._finalize_run()

    # ------------------ 모니터(정체/타임아웃) ------------------
    def _monitor_loop(self):
        """
        진행 정체(stuck) 또는 전체 타임아웃 감지 시 취소 + 재큐잉.
        - stuck: 피드백 갱신(last_progress_ts) 이후 경과가 기준 초과
        - timeout: goal 전송 시각 이후 경과가 기준 초과
        """
        now = time.time()
        for rob in self.robots:
            if not rob.state.busy:
                continue
            no_progress = (now - rob.state.last_progress_ts) > self.stuck_timeout
            timed_out = (now - rob._send_time) > self.goal_timeout
            if no_progress or timed_out:
                reason = "timeout" if timed_out else "stuck"
                self.get_logger().warn(f"[{rob.ns}] {reason} → cancel and requeue")
                rob.cancel()
                if rob.state.goal is not None:
                    g = rob.state.goal
                    t = Task(
                        id=f"requeue_{int(now)}",
                        x=g.pose.position.x,
                        y=g.pose.position.y,
                        yaw_rad=quat_to_yaw(g.pose.orientation),
                        priority=0,
                    )
                    # 빠른 재시도를 위해 앞에 삽입
                    self.task_queue.appendleft(t)
                rob.state.busy = False
                # 즉시 다음 집행 시도
                self._dispatch_loop()

    # ------------------ 충돌/헤드온 휴리스틱 ------------------
    def _will_conflict(self, candidate_robot: RobotAgent, task: Task) -> bool:
        """
        간단한 충돌/헤드온 보류 판단:
        - 후보 로봇의 ETA와 다른 로봇의 ETA가 비슷(±3s)하고
          현재 서로의 거리가 coordination_radius*2 보다 가깝고
          또는 지금 이미 coordination_radius 보다 근접한 경우 보류.
        - 실제 동적 장애물 회피는 Nav2 Controller가 담당. 여기선 '동시 교차'를 피하는 완충.
        """
        if candidate_robot.state.pose is None:
            # 위치를 모르므로 판단 불가 → 보류하지 않음
            return False

        cand_eta = self._eta(candidate_robot.state.pose, task)

        for r in self.robots:
            if r is candidate_robot:
                continue
            if not r.state.busy or r.state.pose is None or r.state.goal is None:
                continue

            # 현재 상호 거리
            now_dist = dist_xy(candidate_robot.state.pose.pose.position, r.state.pose.pose.position)
            if now_dist < self.coord_radius:
                return True  # 너무 가깝다 → 보류

            # 상대 로봇 ETA
            r_eta = self._eta(r.state.pose, pose_to_task(r.state.goal))
            # ETA 유사 + 비교적 근접하면 보류
            if abs(cand_eta - r_eta) < 3.0 and now_dist < (self.coord_radius * 2.0):
                return True

        return False

    def _eta(self, pose: PoseStamped, task: Task) -> float:
        """
        아주 단순한 ETA 근사: 직선거리 / nominal_speed.
        - 실제 경로/장애물/속도제약은 Nav2가 고려.
        - 스케줄링용 근사치로만 사용.
        """
        d = math.hypot(task.x - pose.pose.position.x, task.y - pose.pose.position.y)
        nominal = getattr(self.allocator, "nominal_speed", 0.25)
        return d / max(1e-3, nominal)

    # ------------------ 드롭오프 후보 계산 ------------------
    def _compute_dropoff_task(self, rob: RobotAgent, base_priority: int) -> Optional[Task]:
        """
        현재 로봇 위치를 기준으로 dropoff_mode에 맞는 목표점을 생성.
        - point  : 고정 좌표(x,y,yaw)
        - circle : 중심(cx,cy)와 반지름 r. 원 밖→경계 최근접점, 원 안/중심→중심.
        - rect   : 회전 사각형 내부로 최근접점(클램프) 계산.
        """
        if rob.state.pose is None:
            # 로봇 위치를 모르면 point 모드의 고정값으로 폴백
            x, y, yaw = self.dropoff_x, self.dropoff_y, self.dropoff_yaw
            return Task(
                id=f"deliver_{int(time.time()*1000)}",
                x=x, y=y, yaw_rad=yaw,
                priority=max(base_priority, 9999),  # 드롭오프를 매우 우선 처리
                is_delivery=True,
                preferred_robot=rob.ns,
            )

        rx = rob.state.pose.pose.position.x
        ry = rob.state.pose.pose.position.y

        mode = (self.drop_mode or "point").lower()

        if mode == "point":
            x, y, yaw = self.dropoff_x, self.dropoff_y, self.dropoff_yaw

        elif mode == "circle":
            cx, cy, r = self.drop_cx, self.drop_cy, self.drop_r
            dx, dy = rx - cx, ry - cy
            dist = math.hypot(dx, dy)

            if dist < 1e-6:
                # 중심에 너무 가깝다면 중심을 그대로 목표로
                x, y = cx, cy
            elif dist <= r:
                # 이미 원 내부 → 중심으로 정렬
                x, y = cx, cy
            else:
                # 원 밖 → 로봇-중심 방향으로 원 경계상의 최근접점
                ux, uy = -dx / dist, -dy / dist  # center - robot 방향 단위벡터
                # center + (dx/dist)*r 와 동일. 위에서 ux=-dx/dist라 -ux*r = (dx/dist)*r
                x, y = cx + (-ux) * r, cy + (-uy) * r

            # 접근 방향을 바라보도록 yaw 설정
            yaw = math.atan2(y - ry, x - rx)

        elif mode == "rect":
            # 회전된 사각형 내부로 최근접점(로컬 좌표계에서 클램프)
            cx, cy = self.drop_rx, self.drop_ry
            yaw_r = self.drop_r_yaw
            w, h = self.drop_rw, self.drop_rh

            # 월드 → 로컬(사각형 기준 좌표계)
            lx, ly = world_to_local(rx, ry, cx, cy, yaw_r)
            hx, hy = 0.5 * w, 0.5 * h

            # 내부 최근접점 (클램프)
            clx = clamp(lx, -hx, hx)
            cly = clamp(ly, -hy, hy)

            # 로컬 → 월드 복귀
            x, y = local_to_world(clx, cly, cx, cy, yaw_r)

            # 접근 방향을 바라보도록 yaw 설정
            yaw = math.atan2(y - ry, x - rx)

        else:
            # 알 수 없는 모드 → point 폴백
            x, y, yaw = self.dropoff_x, self.dropoff_y, self.dropoff_yaw

        return Task(
            id=f"deliver_{int(time.time()*1000)}",
            x=x,
            y=y,
            yaw_rad=yaw,
            priority=max(base_priority, 9999),
            is_delivery=True,
            preferred_robot=rob.ns,
        )


# ------------------------------
# 시퀀싱 베이스라인(필수 지점 → 방문 순서 생성)
# ------------------------------
def sequencing_baseline(required_sites: List[Task], robots: List[RobotAgent]) -> List[Task]:
    """
    매우 단순한 시퀀싱: priority 큰 순 → 생성시각 순.
    - 실제로는 다중 로봇/통로 제약/TSP 근사 등을 통합해야 효과적.
    """
    return sorted(required_sites, key=lambda t: (-t.priority, t.created_ts))


# ------------------------------
# 헬퍼 함수
# ------------------------------
def quat_to_yaw(q: Quaternion) -> float:
    """
    Quaternion(z,w)만 사용하는 간단한 yaw 복원.
    - roll/pitch=0 가정에서 정확.
    """
    t3 = 2.0 * (q.w * q.z)
    t4 = 1.0 - 2.0 * (q.z * q.z)
    return math.atan2(t3, t4)

def dist_xy(a: Point, b: Point) -> float:
    """2D 유클리드 거리."""
    return math.hypot(a.x - b.x, a.y - b.y)

def pose_to_task(ps: PoseStamped) -> Task:
    """PoseStamped → Task (ETA 비교 등 내부용)."""
    return Task(
        id="from_pose",
        x=ps.pose.position.x,
        y=ps.pose.position.y,
        yaw_rad=quat_to_yaw(ps.pose.orientation),
        priority=0,
    )

def clamp(v: float, lo: float, hi: float) -> float:
    """값을 [lo, hi]로 제한."""
    return max(lo, min(hi, v))

def world_to_local(px: float, py: float, cx: float, cy: float, yaw: float) -> Tuple[float, float]:
    """
    월드 좌표(px,py) → (cx,cy,yaw) 기준 로컬 좌표.
    - 회전 행렬 R(-yaw) 적용 + 원점 평행이동.
    """
    dx, dy = px - cx, py - cy
    c, s = math.cos(-yaw), math.sin(-yaw)
    return (c * dx - s * dy, s * dx + c * dy)

def local_to_world(lx: float, ly: float, cx: float, cy: float, yaw: float) -> Tuple[float, float]:
    """
    로컬 좌표(lx,ly) → 월드 좌표.
    - 회전 R(yaw) 적용 + 원점 복원.
    """
    c, s = math.cos(yaw), math.sin(yaw)
    return (cx + c * lx - s * ly, cy + s * lx + c * ly)


# ------------------------------
# 진입 포인트
# ------------------------------
def main(args=None):
    """
    rclpy 초기화 후 MultiThreadedExecutor로
    - TaskManager(중앙 조정자)
    - 각 RobotAgent(로봇별 래퍼)
    를 실행.
    """
    rclpy.init(args=args)
    from rclpy.executors import MultiThreadedExecutor

    fm = TaskManager()
    exec = MultiThreadedExecutor()

    exec.add_node(fm)
    for r in fm.robots:
        exec.add_node(r)

    try:
        exec.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 노드 정리 및 종료
        for r in fm.robots:
            exec.remove_node(r)
            r.destroy_node()
        exec.remove_node(fm)
        fm.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
