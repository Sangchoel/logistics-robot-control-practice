#!/usr/bin/env python3
"""
ROS2 플릿 매니저 (Python) — 동작 가능한 베이스라인 (+ 자동 드롭오프 범위 모드)
==========================================================================

여러 대의 Nav2 로봇(TurtleBot3 등)을 중앙에서 제어하기 위한 동작 가능한 참고 구현입니다.
- Nav2 목표 전송(NavigateToPose 액션)
- 작업 큐 & 그리디 할당(ETA 기반 기본 정책)
- 간단한 헤드온(정면 교차) 완화 휴리스틱
- 진행 모니터링(정체/타임아웃 → 취소/재할당)
- 실습/대회용 입력: `required_sites_yaml`(들러야 하는 위치들)
- 드라이런 모드: `dry_run=True`면 Nav2에 실제 goal을 보내지 않고 로직만 검증

※ 강의에서는 Allocator/Sequencing을 교체하거나 TODO로 바꿔 실습 과제
※ 드롭오프 모드:
   - point(기본): 기존 dropoff_x/y/yaw_deg로 고정 지점
   - circle: dropoff_center_x/y + dropoff_radius_m
   - rect: dropoff_rect_cx/cy + dropoff_rect_yaw_deg + dropoff_rect_w/h
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
from std_srvs.srv import SetBool  # 토글 서비스


# ------------------------------
# 데이터 모델
# ------------------------------
@dataclass(order=True)
class Task:
    # 우선순위 큐 정렬 키(내부용)
    sort_index: float = field(init=False, repr=False)

    # 태스크 ID 및 목표 포즈(map 기준)
    id: str
    x: float
    y: float
    yaw_rad: float = 0.0

    # 스케줄링 힌트(선택): 우선순위/데드라인
    priority: int = 0                 # 값이 클수록 먼저 처리
    deadline_sec: Optional[float] = None
    created_ts: float = field(default_factory=lambda: time.time())

    # 드롭오프 연쇄 제어용 표시(전략 로직은 그대로 유지)
    is_delivery: bool = False                 # 드롭오프(내림) 태스크 여부
    preferred_robot: Optional[str] = None     # 이 로봇이 처리하길 선호(같은 로봇 이어서 처리)

    def __post_init__(self):
        # 간단한 우선순위 큐: (-priority, 생성시간)
        self.sort_index = (-self.priority, self.created_ts)

    def to_pose_stamped(self, frame_id: str = "map") -> PoseStamped:
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = Clock().now().to_msg()
        ps.pose.position = Point(x=self.x, y=self.y, z=0.0)
        qx, qy, qz, qw = yaw_to_quat(self.yaw_rad)
        ps.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        return ps


def yaw_to_quat(yaw: float) -> Tuple[float, float, float, float]:
    """yaw(라디안) → 쿼터니언(x,y,z,w). roll=pitch=0 가정."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (0.0, 0.0, sy, cy)


@dataclass
class RobotState:
    # 로봇 네임스페이스
    ns: str
    # 현재 포즈 및 목표 포즈
    pose: Optional[PoseStamped] = None
    goal: Optional[PoseStamped] = None
    # 최근 진행(피드백 기준) 타임스탬프
    last_progress_ts: float = field(default_factory=lambda: time.time())
    # 작업 중 여부/실패 횟수
    busy: bool = False
    failed_count: int = 0


# ------------------------------
# 로봇 에이전트(로봇별 래퍼)
# ------------------------------
class RobotAgent(Node):
    def __init__(self, ns: str):
        super().__init__(f"robot_agent_{ns}", namespace=ns)

        self.ns = ns
        self.state = RobotState(ns=ns)

        # AMCL 포즈 구독(QoS: RELIABLE, KEEP_LAST=10)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            f"/{ns}/amcl_pose",
            self._on_amcl_pose,
            qos
        )

        # Nav2 NavigateToPose 액션 클라이언트(네임스페이스 포함)
        self._nav_client = ActionClient(self, NavigateToPose, f"/{ns}/navigate_to_pose")

        self._goal_handle = None
        self._send_time = 0.0
        # 이동거리 적산기
        self._last_xy: Optional[Tuple[float, float]] = None
        self._dist_m: float = 0.0

        self.get_logger().info(f"RobotAgent for '{ns}' ready → action: /{ns}/navigate_to_pose")
            
    def reset_odometer(self):
        self._dist_m = 0.0
        if self.state.pose is not None:
            p = self.state.pose.pose.position
            self._last_xy = (p.x, p.y)
        else:
            self._last_xy = None

    @property
    def total_distance_m(self) -> float:
        return self._dist_m

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

    async def go_to(self, goal: PoseStamped) -> bool:
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
            feedback_callback=self._feedback_cb
        )
        goal_handle = await send_goal_future
        if not goal_handle.accepted:
            self.get_logger().warn(f"[{self.ns}] Goal rejected by server")
            self.state.busy = False
            return False

        self._goal_handle = goal_handle
        return True

    async def wait_result(self) -> Optional[int]:
        if self._goal_handle is None:
            return None
        result_future = self._goal_handle.get_result_async()
        result = await result_future
        return int(result.status)

    def cancel(self):
        if self._goal_handle is not None:
            self.get_logger().warn(f"[{self.ns}] Canceling current goal…")
            self._goal_handle.cancel_goal_async()

    def _feedback_cb(self, feedback: NavigateToPose.Feedback):  # type: ignore[attr-defined]
        try:
            dist = feedback.feedback.distance_remaining  # type: ignore[attr-defined]
        except Exception:
            dist = None
        if dist is not None:
            self.state.last_progress_ts = time.time()


# ------------------------------
# 작업 할당 전략 (베이스라인: 단순 idle-first; 교육용 그대로 유지)
# ------------------------------
class Allocation:
    def __init__(self, nominal_speed: float = 0.25):
        self.nominal_speed = max(1e-3, nominal_speed)

    def pick(self, robots: List[RobotAgent], task: Task) -> Optional[RobotAgent]:
        for r in robots:
            if not r.state.busy and r.state.pose is not None:
                return r
        for r in robots:
            if not r.state.busy:
                return r
        return None


# ------------------------------
# 플릿 매니저 노드
# ------------------------------
class TaskManager(Node):
    def __init__(self):
        super().__init__("multi_robot_controller")

        # 기본 파라미터
        self.declare_parameter("robot_namespaces", ["tb1", "tb3"])
        self.declare_parameter("nominal_speed_mps", 0.25)
        self.declare_parameter("stuck_timeout_sec", 20.0)
        self.declare_parameter("goal_timeout_sec", 180.0)
        self.declare_parameter("coordination_radius_m", 0.8)
        self.declare_parameter("required_sites_yaml", "")
        self.declare_parameter("dry_run", False)

        # 자동 드롭오프 (토글 + 모드/파라미터)
        self.declare_parameter("auto_deliver_enabled", False)
        self.declare_parameter("dropoff_mode", "point")  # point | circle | rect

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

        # 파라미터 로드
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

        # 토글 서비스
        self.auto_srv = self.create_service(SetBool, "/fleet/auto_deliver", self._srv_set_auto_deliver)

        self.allocator = Allocation(nominal_speed=nominal_speed)

        # 작업 큐
        self.task_queue: Deque[Task] = deque()

        # 로봇 에이전트 생성
        self.robots: List[RobotAgent] = []
        for ns in ns_list:
            self.robots.append(RobotAgent(ns))

        # 신규 작업 입력 토픽(선택사항): /fleet/new_task
        qos = QoSProfile(depth=10)
        self.new_task_sub = self.create_subscription(
            PoseStamped,
            "/fleet/new_task",
            self._on_new_task,
            qos
        )

        # 주기 타이머(디스패치/모니터링)
        self.dispatch_timer = self.create_timer(0.5, self._dispatch_loop)
        self.monitor_timer = self.create_timer(0.5, self._monitor_loop)
        
        # 런 상태
        self._run_started: bool = False
        self._t0: float = 0.0
        self._summary_done: bool = False

        # 필수 지점 로드 → sequencing → 큐 적재
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
        self.auto_deliver = bool(request.data)
        response.success = True
        response.message = f"auto_deliver set to {self.auto_deliver}"
        self.get_logger().warn(response.message)
        return response

    def _start_run(self):
        if self._run_started:
            return
        self._run_started = True
        self._summary_done = False
        self._t0 = time.time()
        for r in self.robots:
            r.reset_odometer()
        self.get_logger().info("Run started: odometers reset and timer started.")

    def _all_done(self) -> bool:
        return (not self.task_queue) and all(not r.state.busy for r in self.robots)

    def _finalize_run(self):
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
        yaw = quat_to_yaw(msg.pose.orientation)
        t = Task(id=f"task_{int(time.time()*1000)}", x=msg.pose.position.x, y=msg.pose.position.y, yaw_rad=yaw)
        self.task_queue.append(t)
        self.get_logger().info(f"Enqueued new task {t.id} at ({t.x:.2f},{t.y:.2f})")

    def _load_required_sites(self, path: str) -> List[Task]:
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

    # ------------------ 디스패처 ------------------
    def _dispatch_loop(self):
        if not self.task_queue:
            return
        task = self.task_queue[0]

        # preferred_robot가 지정되어 있으면 그 로봇이 idle일 때만 집행
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

        # 평소처럼 기존 할당기 사용
        rob = self.allocator.pick(self.robots, task)
        if rob is None:
            return
        if self._will_conflict(rob, task):
            return

        self.task_queue.popleft()
        goal = task.to_pose_stamped()
        self._send_goal_via_callbacks(rob, task, goal)

    def _send_goal_via_callbacks(self, rob: RobotAgent, task: Task, goal: PoseStamped):
        if not self._run_started:
            self._start_run()

        if self.dry_run:
            self.get_logger().info(
                f"[{rob.ns}] (DRY-RUN) would send goal → x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}"
            )
            rob.state.goal = goal
            rob.state.busy = False
            self.get_logger().info(f"[{rob.ns}] (DRY-RUN) Task {task.id} SUCCEEDED")
            self._dispatch_loop()
            return

        if not rob._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"[{rob.ns}] NavigateToPose action server not available")
            rob.state.failed_count += 1
            self.task_queue.append(task)
            self._dispatch_loop()
            return

        rob.state.goal = goal
        rob.state.busy = True
        rob._send_time = time.time()
        self.get_logger().info(
            f"[{rob.ns}] Sending goal → x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}"
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal

        send_future = rob._nav_client.send_goal_async(goal_msg, feedback_callback=rob._feedback_cb)

        def _goal_response_cb(fut):
            try:
                goal_handle = fut.result()
            except Exception as e:
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

        self._dispatch_loop()
        if self._run_started and self._all_done():
            self._finalize_run()

    # ------------------ 모니터 ------------------
    def _monitor_loop(self):
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
                    self.task_queue.appendleft(t)
                rob.state.busy = False
                self._dispatch_loop()

    # ------------------ 충돌 휴리스틱 ------------------
    def _will_conflict(self, candidate_robot: RobotAgent, task: Task) -> bool:
        if candidate_robot.state.pose is None:
            return False
        cand_eta = self._eta(candidate_robot.state.pose, task)
        for r in self.robots:
            if r is candidate_robot:
                continue
            if not r.state.busy or r.state.pose is None or r.state.goal is None:
                continue
            now_dist = dist_xy(candidate_robot.state.pose.pose.position, r.state.pose.pose.position)
            if now_dist < self.coord_radius:
                return True
            r_eta = self._eta(r.state.pose, pose_to_task(r.state.goal))
            if abs(cand_eta - r_eta) < 3.0 and now_dist < (self.coord_radius * 2.0):
                return True
        return False

    def _eta(self, pose: PoseStamped, task: Task) -> float:
        d = math.hypot(task.x - pose.pose.position.x, task.y - pose.pose.position.y)
        nominal = getattr(self.allocator, "nominal_speed", 0.25)
        return d / max(1e-3, nominal)

    # ------------------ 드롭오프 후보 계산 ------------------
    def _compute_dropoff_task(self, rob: RobotAgent, base_priority: int) -> Optional[Task]:
        """현재 로봇 위치를 기준으로 dropoff_mode에 맞는 목표점을 계산해 Task 생성."""
        if rob.state.pose is None:
            # 로봇 위치를 모르면 point 모드의 고정값으로 폴백
            x, y, yaw = self.dropoff_x, self.dropoff_y, self.dropoff_yaw
            return Task(
                id=f"deliver_{int(time.time()*1000)}",
                x=x, y=y, yaw_rad=yaw,
                priority=max(base_priority, 9999),
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
                # 로봇이 정확히 중심에 있으면 살짝 오프셋
                x, y = cx + r * 0.0, cy + r * 0.0
                x, y = cx, cy  # 중심
            elif dist <= r:
                # 이미 원 안이면 중심으로 유도(결정적 목표)
                x, y = cx, cy
            else:
                # 원 밖이면, 로봇→중심 방향으로 원 경계 최근접점
                ux, uy = -dx / dist, -dy / dist  # center - robot 방향 단위벡터
                x, y = cx + (-ux) * r, cy + (-uy) * r  # 또는 cx + (dx/dist)*r, cy + (dy/dist)*r

            yaw = math.atan2(y - ry, x - rx)  # 접근 방향을 바라보게

        elif mode == "rect":
            cx, cy = self.drop_rx, self.drop_ry
            yaw_r = self.drop_r_yaw
            w, h = self.drop_rw, self.drop_rh
            # 로컬 좌표로 변환(사각형 yaw 기준)
            lx, ly = world_to_local(rx, ry, cx, cy, yaw_r)
            # 반너비/반높이
            hx, hy = 0.5 * w, 0.5 * h
            # 로봇 위치를 사각형 내부로 클램프 → 내부 최근접점
            clx = clamp(lx, -hx, hx)
            cly = clamp(ly, -hy, hy)
            # 로컬→월드 복귀
            x, y = local_to_world(clx, cly, cx, cy, yaw_r)
            # 접근 방향
            yaw = math.atan2(y - ry, x - rx)

        else:
            # 알 수 없는 모드 → point로 폴백
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
    return sorted(required_sites, key=lambda t: (-t.priority, t.created_ts))


# ------------------------------
# 헬퍼 함수
# ------------------------------
def quat_to_yaw(q: Quaternion) -> float:
    t3 = 2.0 * (q.w * q.z)
    t4 = 1.0 - 2.0 * (q.z * q.z)
    return math.atan2(t3, t4)

def dist_xy(a: Point, b: Point) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)

def pose_to_task(ps: PoseStamped) -> Task:
    return Task(
        id="from_pose",
        x=ps.pose.position.x,
        y=ps.pose.position.y,
        yaw_rad=quat_to_yaw(ps.pose.orientation),
        priority=0,
    )

def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))

def world_to_local(px: float, py: float, cx: float, cy: float, yaw: float) -> Tuple[float, float]:
    """월드→로컬(사각형 중심/각도 기준)"""
    dx, dy = px - cx, py - cy
    c, s = math.cos(-yaw), math.sin(-yaw)
    return (c * dx - s * dy, s * dx + c * dy)

def local_to_world(lx: float, ly: float, cx: float, cy: float, yaw: float) -> Tuple[float, float]:
    """로컬→월드(사각형 중심/각도 기준)"""
    c, s = math.cos(yaw), math.sin(yaw)
    return (cx + c * lx - s * ly, cy + s * lx + c * ly)


# ------------------------------
# 엔트리 포인트
# ------------------------------
def main(args=None):
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
