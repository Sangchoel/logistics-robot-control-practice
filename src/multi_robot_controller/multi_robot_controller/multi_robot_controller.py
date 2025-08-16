#!/usr/bin/env python3
"""
ROS2 플릿 매니저 (Python) — 동작 가능한 베이스라인
===============================================

여러 대의 Nav2 로봇(TurtleBot3 등)을 중앙에서 제어하기 위한 동작 가능한 구현입니다.
- Nav2 목표 전송(NavigateToPose 액션)
- 작업 큐 & 그리디 할당
- 간단한 헤드온(정면 교차) 완화 휴리스틱
- 진행 모니터링(정체/타임아웃 → 취소/재할당)
- 실습/대회용 입력: `required_sites_yaml`(반드시 들러야 하는 위치들)
- 드라이런 모드: `dry_run=True`면 Nav2에 실제 goal을 보내지 않고 로직만 검증

※ 강의에서는 Allocator/Sequencing을 교체가능
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Deque, List, Optional, Tuple
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


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

    def __post_init__(self):
        # 간단한 우선순위 큐: (-priority, 생성시간)
        self.sort_index = (-self.priority, self.created_ts)

    def to_pose_stamped(self, frame_id: str = "map") -> PoseStamped:
        # map 프레임 기준 PoseStamped 생성
        ps = PoseStamped()
        ps.header.frame_id = frame_id
        ps.header.stamp = rclpy.clock.Clock().now().to_msg()
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
        #--- 이동거리 적산기 ---#
        self._last_xy: Optional[Tuple[float, float]] = None
        self._dist_m: float = 0.0

        self.get_logger().info(f"RobotAgent for '{ns}' ready → action: /{ns}/navigate_to_pose")
            
    # --- [추가] odom-meter 제로 및 기준 위치 세팅 ---
    def reset_odometer(self):
        self._dist_m = 0.0
        if self.state.pose is not None:
            p = self.state.pose.pose.position
            self._last_xy = (p.x, p.y)
        else:
            self._last_xy = None

    # --- [추가] 읽기 편하도록 프로퍼티 제공 ---
    @property
    def total_distance_m(self) -> float:
        return self._dist_m

    # --- 센싱 콜백 ---
    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        # AMCL 포즈를 PoseStamped로 변환하여 상태에 저장 -> map 기준 로봇 위치 반환기
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose

        # --- [추가] 이동거리 적산 ---
        x, y = ps.pose.position.x, ps.pose.position.y
        if self._last_xy is not None:
            dx = x - self._last_xy[0]
            dy = y - self._last_xy[1]
            self._dist_m += math.hypot(dx, dy)
        self._last_xy = (x, y)

        self.state.pose = ps


    # --- 목표 디스패치 ---
    async def go_to(self, goal: PoseStamped) -> bool:
        """Nav2 NavigateToPose 목표 전송. 서버가 수락하면 True 반환."""
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
        # 결과 코드: GoalStatus 값
        if self._goal_handle is None:
            return None
        result_future = self._goal_handle.get_result_async()
        result = await result_future
        return int(result.status)

    def cancel(self):
        # 현재 목표를 취소(비동기)
        if self._goal_handle is not None:
            self.get_logger().warn(f"[{self.ns}] Canceling current goal…")
            self._goal_handle.cancel_goal_async()

    # --- 피드백 콜백 ---
    def _feedback_cb(self, feedback: NavigateToPose.Feedback):  # type: ignore[attr-defined]
        """Nav2 피드백 수신 시, 남은 거리 감소를 근거로 진행 타임스탬프 갱신."""
        try:
            dist = feedback.feedback.distance_remaining  # type: ignore[attr-defined]
        except Exception:
            dist = None
        if dist is not None:
            self.state.last_progress_ts = time.time()


# ------------------------------
# 작업 할당 전략 (베이스라인: ETA 그리디)
# ------------------------------
class Allocation:
    """
    베이스라인 할당기: 사용 가능 로봇 중 ETA(직선거리/명목속도)가 최소인 로봇을 선택.
    강의/대회에서는 이 클래스를 교체/수정하여 성능 개선 가능.
    """
    def __init__(self, nominal_speed: float = 0.25):
        self.nominal_speed = max(1e-3, nominal_speed)

    # def pick(self, robots: List[RobotAgent], task: Task) -> Optional[RobotAgent]:
    #     # 1) idle 로봇만 후보
    #     available = [r for r in robots if not r.state.busy]
    #     if not available:
    #         return None
    #     # 2) 후보 정렬: (포즈 유무 우선, 실패횟수 적은 순) → 동일하면 네임스페이스 사전순
    #     available.sort(key=lambda r: (r.state.pose is None, r.state.failed_count, r.ns))
    #     # 3) ETA 비용으로 최종 선택(포즈 없는 로봇은 큰 비용)
    #     best = None
    #     best_cost = float("inf")
    #     for r in available:
    #         if r.state.pose is None:
    #             cost = 1e6  # 위치 미확인: 후순위로 크게 패널티
    #         else:
    #             dx = r.state.pose.pose.position.x - task.x
    #             dy = r.state.pose.pose.position.y - task.y
    #             d = math.hypot(dx, dy)
    #             cost = d / self.nominal_speed
    #         if cost < best_cost:
    #             best_cost = cost
    #             best = r
    #     return best
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

        # 파라미터 선언
        self.declare_parameter("robot_namespaces", ["tb1", "tb3"])  # 터미널 명령 입력시 namespace 입력 없으면 기본적으로 1,3 으로 작동
        self.declare_parameter("nominal_speed_mps", 0.25) #속도 : 임의 변경 가능
        self.declare_parameter("stuck_timeout_sec", 20.0) # 진행없음 판정시간
        self.declare_parameter("goal_timeout_sec", 180.0) # 전체 타임아웃
        self.declare_parameter("coordination_radius_m", 0.8) # 헤드온 가능성 방지를 위한 파라미터
        # 실습/대회용 입력
        self.declare_parameter("required_sites_yaml", "")  # 반드시 들러야 하는 위치들
        self.declare_parameter("dry_run", False)            # True: Nav2에 goal 미전송 테스트용임

        # 파라미터 로드
        ns_list = self.get_parameter("robot_namespaces").get_parameter_value().string_array_value
        nominal_speed = self.get_parameter("nominal_speed_mps").get_parameter_value().double_value
        self.stuck_timeout = self.get_parameter("stuck_timeout_sec").get_parameter_value().double_value
        self.goal_timeout = self.get_parameter("goal_timeout_sec").get_parameter_value().double_value
        self.coord_radius = self.get_parameter("coordination_radius_m").get_parameter_value().double_value
        self.dry_run = self.get_parameter("dry_run").get_parameter_value().bool_value

        self.allocator = Allocation(nominal_speed=nominal_speed)

        # 작업 큐
        self.task_queue: Deque[Task] = deque()

        # 로봇 에이전트 생성
        self.robots: List[RobotAgent] = []
        for ns in ns_list:
            self.robots.append(RobotAgent(ns))

        # 신규 작업 입력 토픽(선택사항): /fleet/new_task 를 통해 작업 추가 가능
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
        
        # --- [추가] 런 타이밍/상태 ---
        self._run_started: bool = False
        self._t0: float = 0.0
        self._summary_done: bool = False

        # 실습 시나리오 입력: required_sites_yaml 로드 → sequencing → 큐 적재(pq 기반 우선순위 설정 가능 기본적으로 전부 1로 설정)
        sites_path = self.get_parameter("required_sites_yaml").get_parameter_value().string_value
        if sites_path:
            sites = self._load_required_sites(sites_path)
            ordered_tasks = sequencing_baseline(sites, self.robots)
            for t in ordered_tasks:
                self.task_queue.append(t)
            self.get_logger().info(f"필수 지점 {len(ordered_tasks)}건 큐 적재 완료")

        self.get_logger().info(
            f"FleetManager up with robots: {', '.join(ns_list)} | dry_run={self.dry_run}"
        )

    # --- [추가] 런 시작: 첫 디스패치 직전에 호출 ---
    def _start_run(self):
        if self._run_started:
            return
        self._run_started = True
        self._summary_done = False
        self._t0 = time.time()
        for r in self.robots:
            r.reset_odometer()
        self.get_logger().info("Run started: odometers reset and timer started.")

    # --- [추가] 모두 끝났는지 검사 ---
    def _all_done(self) -> bool:
        return (not self.task_queue) and all(not r.state.busy for r in self.robots)

    # --- [추가] 요약 출력 & 런 상태 초기화 ---
    def _finalize_run(self):
        if self._summary_done:
            return
        elapsed = time.time() - self._t0
        per_robot = [(r.ns, r.total_distance_m) for r in self.robots]
        total_dist = sum(d for _, d in per_robot)

        self.get_logger().info("=== Fleet Summary (current batch) ===")
        self.get_logger().info(f"Elapsed: {elapsed:.2f} s")
        for ns, d in per_robot:
            self.get_logger().info(f"  {ns}: {d:.3f} m")
        self.get_logger().info(f"Total distance: {total_dist:.3f} m")
        self.get_logger().info("====================================")

        # 다음 배치를 대비해 상태 리셋
        self._summary_done = True
        self._run_started = False
        self._t0 = 0.0


    # --- 작업 입력 핸들러 ---
    def _on_new_task(self, msg: PoseStamped):
        # PoseStamped(map) → Task로 변환 후 큐에 삽입
        yaw = quat_to_yaw(msg.pose.orientation)
        t = Task(id=f"task_{int(time.time()*1000)}", x=msg.pose.position.x, y=msg.pose.position.y, yaw_rad=yaw)
        self.task_queue.append(t)
        self.get_logger().info(f"Enqueued new task {t.id} at ({t.x:.2f},{t.y:.2f})")

    def _load_required_sites(self, path: str) -> List[Task]:
        """필수 방문 지점을 YAML에서 읽어 Task 리스트로 반환(제공)."""
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

    # --- 메인 루프: 디스패치 ---
    def _dispatch_loop(self):
        if not self.task_queue:
            return
        task = self.task_queue[0]
        rob = self.allocator.pick(self.robots, task)
        if rob is None:
            return
        if self._will_conflict(rob, task):
            return
        # 할당 확정
        self.task_queue.popleft()
        goal = task.to_pose_stamped()
        # 비동기는 콜백 체인으로 처리
        self._send_goal_via_callbacks(rob, task, goal)

    # --- Nav2 전송/결과 처리(콜백 체인) ---
    def _send_goal_via_callbacks(self, rob: RobotAgent, task: Task, goal: PoseStamped):
        """Nav2 액션을 콜백 체인으로 전송/결과 처리. asyncio 비사용."""
        # DRY-RUN: Nav2 전송 없이 즉시 완료 처리
        if not self._run_started:
            self._start_run()

        if self.dry_run:
            self.get_logger().info(
                f"[{rob.ns}] (DRY-RUN) would send goal → x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}"
            )
            # 즉시 완료 처리 후 다음 디스패치 바로 시도
            rob.state.goal = goal
            rob.state.busy = False
            self.get_logger().info(f"[{rob.ns}] (DRY-RUN) Task {task.id} SUCCEEDED")
            self._dispatch_loop()
            return

        # 액션 서버 체크
        if not rob._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"[{rob.ns}] NavigateToPose action server not available")
            # 실패 카운트 증가 및 작업 뒤로 재배치(다른 로봇에 기회)
            rob.state.failed_count += 1
            self.task_queue.append(task)
            self._dispatch_loop()
            return

        # 상태 업데이트 및 로그
        rob.state.goal = goal
        rob.state.busy = True
        rob._send_time = time.time()
        self.get_logger().info(
            f"[{rob.ns}] Sending goal → x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}"
        )

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal

        # 1) 목표 전송
        send_future = rob._nav_client.send_goal_async(goal_msg, feedback_callback=rob._feedback_cb)

        # 2) goal_handle 수신 콜백
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
                self.task_queue.append(task)  # 뒤로 밀어 다른 로봇 기회 포함
                self._dispatch_loop()
                return

            rob._goal_handle = goal_handle

            # 3) 결과 대기 콜백
            res_future = goal_handle.get_result_async()
            res_future.add_done_callback(lambda rf: self._on_result_cb(rf, rob, task))

        send_future.add_done_callback(_goal_response_cb)

    def _on_result_cb(self, result_future, rob: RobotAgent, task: Task):
        """액션 결과 콜백."""
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
        elif code == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f"[{rob.ns}] Task {task.id} CANCELED; requeuing…")
            rob.state.failed_count += 1
            self.task_queue.append(task)
        else:  # ABORTED 등
            self.get_logger().warn(f"[{rob.ns}] Task {task.id} ABORTED(code={code}); requeuing…")
            rob.state.failed_count += 1
            self.task_queue.append(task)

        # 어떤 경우든 다음 작업 즉시 시도(상태 변화 기반 트리거)
        self._dispatch_loop()
        
        # --- [추가] 모두 성공 종료됐으면 요약 출력 ---
        if self._run_started and self._all_done():
            self._finalize_run()

    # --- 메인 루프: 모니터링 ---
    def _monitor_loop(self):
        now = time.time()
        for rob in self.robots:
            if not rob.state.busy:
                continue
            # 정체/타임아웃 감지(피드백 기반 진행도 갱신 시간과 송신 후 경과 시간 사용)
            no_progress = (now - rob.state.last_progress_ts) > self.stuck_timeout
            timed_out = (now - rob._send_time) > self.goal_timeout
            if no_progress or timed_out:
                reason = "timeout" if timed_out else "stuck"
                self.get_logger().warn(f"[{rob.ns}] {reason} → cancel and requeue")
                rob.cancel()
                # 현재 목표를 다시 작업으로 변환해 재큐잉
                if rob.state.goal is not None:
                    g = rob.state.goal
                    t = Task(
                        id=f"requeue_{int(now)}",
                        x=g.pose.position.x,
                        y=g.pose.position.y,
                        yaw_rad=quat_to_yaw(g.pose.orientation),
                        priority=0,
                    )
                    self.task_queue.appendleft(t)  # 곧바로 재시도 기회 부여
                rob.state.busy = False
                # 상태 변화 기반 즉시 디스패치 시도
                self._dispatch_loop()

    # --- 간단 충돌(헤드온) 예측 휴리스틱 ---
    def _will_conflict(self, candidate_robot: RobotAgent, task: Task) -> bool:
        """직선 거리/ETA 기반으로 근시안적 충돌 가능성 판단. 가능성이 있으면 이번 주기 디스패치 보류."""
        if candidate_robot.state.pose is None:
            return False
        cand_eta = self._eta(candidate_robot.state.pose, task)
        for r in self.robots:
            if r is candidate_robot:
                continue
            if not r.state.busy or r.state.pose is None or r.state.goal is None:
                continue
            # 현재 거리 및 각자의 ETA를 비교해 비슷한 타이밍에 근접할 것 같으면 지연
            now_dist = dist_xy(candidate_robot.state.pose.pose.position, r.state.pose.pose.position)
            if now_dist < self.coord_radius:
                return True
            r_eta = self._eta(r.state.pose, pose_to_task(r.state.goal))
            if abs(cand_eta - r_eta) < 3.0 and now_dist < (self.coord_radius * 2.0):
                return True
        return False

    def _eta(self, pose: PoseStamped, task: Task) -> float:
        # 직선 거리 / 명목 속도 = ETA(초)
        d = math.hypot(task.x - pose.pose.position.x, task.y - pose.pose.position.y)
        # Allocator와 같은 nominal_speed를 사용(간소화)
        nominal = getattr(self.allocator, "nominal_speed", 0.25)
        return d / max(1e-3, nominal)


# ------------------------------
# 시퀀싱 베이스라인(필수 지점 → 방문 순서 생성)
# ------------------------------

def sequencing_baseline(required_sites: List[Task], robots: List[RobotAgent]) -> List[Task]:
    """
    베이스라인: 우선순위(priority) 내림차순으로 정렬한 뒤, 그대로 작업 큐 순서로 사용.
    - 간단하지만 즉시 동작하며, 참가자는 이 부분을 개선하여 성능을 높일 수 있음.
    - 개선 예: 각 로봇 출발 위치를 고려한 최근접 삽입, TSP 근사, 데드라인/혼잡도 반영 등.
    """
    return sorted(required_sites, key=lambda t: (-t.priority, t.created_ts))


# ------------------------------
# 헬퍼 함수 이 함수들을 사용하여 sequencing, allocator 개선 할수있음음
# ------------------------------

def quat_to_yaw(q: Quaternion) -> float:
    """쿼터니언(z, w만 사용) → yaw 라디안. roll=pitch=0 가정."""
    t3 = 2.0 * (q.w * q.z)
    t4 = 1.0 - 2.0 * (q.z * q.z)
    return math.atan2(t3, t4)


def dist_xy(a: Point, b: Point) -> float:
    """2D 평면 유클리드 거리."""
    return math.hypot(a.x - b.x, a.y - b.y)


def pose_to_task(ps: PoseStamped) -> Task:
    """PoseStamped → Task 변환(ETA 계산 등 내부 용도)."""
    return Task(
        id="from_pose",
        x=ps.pose.position.x,
        y=ps.pose.position.y,
        yaw_rad=quat_to_yaw(ps.pose.orientation),
        priority=0,
    )


# ------------------------------
# 엔트리 포인트
# ------------------------------

def main(args=None):
    rclpy.init(args=args)

    # 멀티스레드 실행기: 여러 RobotAgent 노드를 동시에 스핀
    from rclpy.executors import MultiThreadedExecutor

    fm = TaskManager()
    exec = MultiThreadedExecutor()

    # 플릿 매니저 및 로봇 노드 등록
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
