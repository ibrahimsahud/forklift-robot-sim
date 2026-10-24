#!/usr/bin/env python3
"""
diagnose_nav2_goal.py — تشغيل مباشر بدون setup.py
python3 src/articubot_one/articubot_one/diagnose_nav2_goal.py

يثبت شيئاً واحداً فقط:
  هل Nav2 يصل إلى A1_STAGING_PRE في بيئة نظيفة؟

المتطلبات قبل التشغيل:
  - localization_navigation.launch.py فقط (بدون route_executor / line / QR)
  - لا عمليات مكررة
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist, TwistStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus
from std_msgs.msg import Header
import yaml, math, sys, time
from pathlib import Path

# ════════════════════════════════════════════
# ضع هنا مسار ملف route_waypoints.yaml
WAYPOINTS_FILE = str(
    Path(__file__).resolve().parents[1] / "config" / "route_waypoints.yaml"
)
TARGET_NAME    = "A1_STAGING_PRE"
TIMEOUT_SEC    = 60.0
# ════════════════════════════════════════════


class NavDiagnostic(Node):
    def __init__(self, target: dict):
        super().__init__('diagnose_nav2_goal')
        self.target = target

        # ── counters ──────────────────────────────────
        self.nonzero_nav      = 0   # /cmd_vel_nav_smoothed   (Twist)
        self.nonzero_unstamp  = 0   # /diff_cont/cmd_vel_unstamped (Twist)
        self.nonzero_final    = 0   # /diff_cont/cmd_vel      (TwistStamped !)
        self.zero_total       = 0

        self.odom_pos   = None
        self.amcl_pos   = None
        self.start_odom = None
        self._send_time = None

        # ── subscriptions ────────────────────────────
        # Twist
        self.create_subscription(
            Twist, '/cmd_vel_nav_smoothed',
            lambda m: self._rec_twist('nav_smoothed', m), 10)

        # Twist
        self.create_subscription(
            Twist, '/diff_cont/cmd_vel_unstamped',
            lambda m: self._rec_twist('unstamped', m), 10)

        # ★ TwistStamped — الفرق الأساسي عن النسخة السابقة
        self.create_subscription(
            TwistStamped, '/diff_cont/cmd_vel',
            lambda m: self._rec_twist_stamped('final', m), 10)

        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)

        # ── action client ────────────────────────────
        self._client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.get_logger().info('[DIAG] انتظار Nav2 action server...')
        if not self._client.wait_for_server(timeout_sec=20.0):
            self.get_logger().error('[DIAG] ❌ Nav2 action server لم يظهر خلال 20s')
            self._print_summary('NO_SERVER')
            rclpy.shutdown()
            return

        self.get_logger().info(f'[DIAG] ✅ Nav2 جاهز — إرسال هدف: {TARGET_NAME}')
        self.get_logger().info(
            f'[DIAG]    x={self.target["x"]:.3f}  y={self.target["y"]:.3f}')
        self._send_goal()

    # ── callbacks: Twist ──────────────────────────────
    def _rec_twist(self, name: str, msg: Twist):
        lin = msg.linear.x
        ang = msg.angular.z
        if abs(lin) > 0.001 or abs(ang) > 0.001:
            if name == 'nav_smoothed':
                self.nonzero_nav += 1
                if self.nonzero_nav <= 3:
                    self.get_logger().info(
                        f'[CMD/{name}] lin={lin:.3f} ang={ang:.3f}')
            else:
                self.nonzero_unstamp += 1
                if self.nonzero_unstamp <= 3:
                    self.get_logger().info(
                        f'[CMD/{name}] lin={lin:.3f} ang={ang:.3f}')
        else:
            self.zero_total += 1

    # ── callback: TwistStamped ────────────────────────
    def _rec_twist_stamped(self, name: str, msg: TwistStamped):
        lin = msg.twist.linear.x    # ★ msg.twist.linear  وليس msg.linear
        ang = msg.twist.angular.z
        if abs(lin) > 0.001 or abs(ang) > 0.001:
            self.nonzero_final += 1
            if self.nonzero_final <= 3:
                self.get_logger().info(
                    f'[CMD/{name}] lin={lin:.3f} ang={ang:.3f}')
        else:
            self.zero_total += 1

    # ── odom / amcl ───────────────────────────────────
    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.odom_pos = (p.x, p.y)
        if self.start_odom is None:
            self.start_odom = (p.x, p.y)

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        p = msg.pose.pose.position
        self.amcl_pos = (p.x, p.y)

    # ── send goal ─────────────────────────────────────
    def _send_goal(self):
        from geometry_msgs.msg import PoseStamped

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = float(self.target['x'])
        goal_msg.pose.pose.position.y = float(self.target['y'])
        goal_msg.pose.pose.orientation.w = 1.0

        self._send_time = time.time()
        fut = self._client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb)
        fut.add_done_callback(self._goal_response_cb)

    def _feedback_cb(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().info(
            f'[FEEDBACK] المسافة المتبقية: {dist:.2f} m',
            throttle_duration_sec=4.0)

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('[RESULT] ❌ الهدف رُفض من Nav2')
            self._print_summary('REJECTED')
            rclpy.shutdown()
            return
        self.get_logger().info('[RESULT] الهدف قُبل ✅ — انتظار النتيجة...')
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        result   = future.result()
        status   = result.status
        elapsed  = time.time() - self._send_time

        dist_moved    = 0.0
        dist_to_goal  = 0.0

        if self.start_odom and self.odom_pos:
            dx = self.odom_pos[0] - self.start_odom[0]
            dy = self.odom_pos[1] - self.start_odom[1]
            dist_moved = math.sqrt(dx*dx + dy*dy)

        if self.odom_pos:
            dx = self.odom_pos[0] - self.target['x']
            dy = self.odom_pos[1] - self.target['y']
            dist_to_goal = math.sqrt(dx*dx + dy*dy)

        label = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_ABORTED:   'ABORTED',
            GoalStatus.STATUS_CANCELED:  'CANCELED',
        }.get(status, f'UNKNOWN({status})')

        self._print_summary(label, elapsed, dist_moved, dist_to_goal)
        rclpy.shutdown()

    # ── final report ──────────────────────────────────
    def _print_summary(self, status, elapsed=0.0, moved=0.0, dist_to=0.0):
        sep = "=" * 58
        print(f"\n{sep}")
        print(f"  نتيجة Nav2  →  {TARGET_NAME}")
        print(sep)
        print(f"  الحالة النهائية         : {status}")
        print(f"  الوقت المستغرق          : {elapsed:.1f} s")
        print(f"  المسافة المقطوعة (odom) : {moved:.3f} m")
        print(f"  المسافة من الهدف        : {dist_to:.3f} m")
        print(f"  cmd_vel_nav_smoothed ≠0 : {self.nonzero_nav}")
        print(f"  cmd_vel_unstamped   ≠0  : {self.nonzero_unstamp}")
        print(f"  cmd_vel (stamped)   ≠0  : {self.nonzero_final}")
        print(f"  cmd_vel صفرية (كل)     : {self.zero_total}")
        print(f"  آخر موقع odom           : {self.odom_pos}")
        print(f"  آخر موقع amcl           : {self.amcl_pos}")
        print(sep)

        # ── تشخيص تلقائي ──────────────────────────────
        if status == 'SUCCEEDED':
            print("\n✅ Nav2 نجح في الوصول لـ A1_STAGING_PRE")
            print("   → المشكلة ليست في Nav2 transit")
            print("   → التالي: فحص line_follower / route logic")

        elif status == 'NO_SERVER':
            print("\n❌ Nav2 action server لم يبدأ")
            print("   → تحقق من: localization_navigation.launch.py")
            print("   → تحقق من: bt_navigator / controller_server / planner_server")

        elif status == 'REJECTED':
            print("\n❌ الهدف رُفض قبل البدء")
            print("   → المحتمل: Nav2 في حالة خطأ أو لم يتم تهيئته")
            print("   → نفّذ: ros2 node info /bt_navigator")

        elif status == 'ABORTED':
            nav_ok   = self.nonzero_nav > 0
            chain_ok = self.nonzero_final > 0

            if not nav_ok:
                print("\n❌ ABORTED — cmd_vel_nav_smoothed = صفر دائماً")
                print("   → المشكلة: Planner أو Controller لم ينتجا أوامر")
                print("   → فحص: planner_server / controller_server logs")
            elif nav_ok and not chain_ok:
                print("\n❌ ABORTED — Nav2 ينتج أوامر لكن لا تصل للموتور")
                print("   → المشكلة: سلسلة twist_mux → twist_stamper → diff_cont")
                print("   → نفّذ: ros2 topic echo /diff_cont/cmd_vel_unstamped")
                print("   → تحقق: twist_mux priority لـ /cmd_vel_nav_smoothed")
            elif moved < 0.05:
                print(f"\n❌ ABORTED — تحرك {moved:.3f}m فقط (أقل من 5cm)")
                print("   → المشكلة: footprint / costmap تمنع الحركة")
                print("   → أو: progress_checker timeout قصير جداً")
                print("   → تحقق: robot_radius في nav2_params.yaml")
            else:
                print(f"\n❌ ABORTED — تحرك {moved:.2f}m ثم توقف")
                print(f"   → المسافة من الهدف: {dist_to:.3f}m")
                print("   → المشكلة: progress_checker أو goal_checker tolerance")
                print("   → تحقق: xy_goal_tolerance / yaw_goal_tolerance في nav2_params.yaml")

        print()


# ════════════════════════════════════════════════════
def load_waypoints():
    path = Path(WAYPOINTS_FILE)
    if not path.exists():
        print(f"[ERROR] ملف الـ waypoints غير موجود:\n  {WAYPOINTS_FILE}")
        sys.exit(1)
    with open(path) as f:
        data = yaml.safe_load(f)

    # يدعم صيغتين:
    #   A1_STAGING_PRE: {x: 1.0, y: 2.0}
    #   waypoints: {A1_STAGING_PRE: {x: 1.0, y: 2.0}}
    if 'waypoints' in data:
        data = data['waypoints']

    return {k: v for k, v in data.items()
            if isinstance(v, dict) and 'x' in v and 'y' in v}


def main():
    rclpy.init()

    waypoints = load_waypoints()
    if TARGET_NAME not in waypoints:
        print(f"[ERROR] '{TARGET_NAME}' غير موجود في الملف.")
        print(f"النقاط المتاحة: {list(waypoints.keys())}")
        sys.exit(1)

    target = waypoints[TARGET_NAME]
    print(f"[DIAG] الهدف: {TARGET_NAME}  x={target['x']}  y={target['y']}")

    node = NavDiagnostic(target)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[DIAG] توقف بواسطة المستخدم")
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
