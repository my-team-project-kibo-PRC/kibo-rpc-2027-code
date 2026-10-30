#!/usr/bin/python3
# -*- coding:utf-8 -*-
import json
import math
import itertools
import rospy
import actionlib

from platform_msgs.msg import UserLogic
from platform_msgs.srv import StopProcessingUserNode, StopProcessingUserNodeResponse
from geometry_msgs.msg import PoseStamped
from krpc_ib2_msgs.srv import MissionStart, MissionFinished, CheckPoints
from krpc_ib2_msgs.msg import CheckpointPassedAction, CheckpointPassedGoal
from ib2_msgs.msg import CtlCommandAction, CtlCommandGoal, CtlCommandResult, CtlStatusType
from ib2_msgs.msg import Navigation


# ---- time model (calibrated from Web simulator runs, simulation seconds) ----
HOP_BASE = 21.0      # fixed seconds per move (acceleration, deceleration, settling)
HOP_PER_M = 5.7      # seconds per metre
START_EXTRA = 6.0    # extra time of the very first move
SETTLE = 0.3         # wait before notifying a normal checkpoint
GOAL_SETTLE = 0.5    # wait before notifying the goal
PLAN_LIMIT = 255.0
PLAN_LIMIT_ALL = 282.0
MOVE_TRIES = 3
REFINE_LIMIT = 18.0   # re-send the move when (att_err + 114.6*pos_err) is above this (deg)
REFINE_WAIT = 2.0     # seconds to wait first, the robot may still be rotating
REFINE_COST = 15.0    # estimated seconds of one extra move
GUARD_LIMIT = 288.0  # run-time guard: never plan to finish later than this
HOP_SCALE = 1.08     # safety factor (Hard runs were ~8% slower than the model)
MISSION_T0 = 7.0     # simulation clock when the mission starts (about 7 s)


def _dist(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def _hop(a, b):
    return HOP_SCALE * (HOP_BASE + HOP_PER_M * _dist(a, b))


def _est_time(start, order, goal_pos):
    t = START_EXTRA
    cur = start
    for c in order:
        t += _hop(cur, c['position']) + SETTLE
        cur = c['position']
    if goal_pos is not None:
        t += _hop(cur, goal_pos) + GOAL_SETTLE
    return t


def _score(n_checkpoints, t, has_goal, limit):
    if t > limit:
        return 0.0
    raw = 10.0 * n_checkpoints + (30.0 if has_goal else 0.0)
    return raw * max(1.0, 2.0 - t / 300.0)


def _best_order(start, cps, goal_pos):
    """Shortest path start -> all cps -> goal (exact for <= 9 points)."""
    n = len(cps)
    if n <= 1:
        return list(cps)
    pos = [c['position'] for c in cps]
    if n <= 12:
        INF = float('inf')
        full = (1 << n) - 1
        dp = [[INF] * n for _ in range(full + 1)]
        par = [[-1] * n for _ in range(full + 1)]
        for j in range(n):
            dp[1 << j][j] = _dist(start, pos[j])
        for mask in range(1, full + 1):
            for j in range(n):
                if dp[mask][j] == INF or not (mask >> j) & 1:
                    continue
                for k in range(n):
                    if (mask >> k) & 1:
                        continue
                    nm = mask | (1 << k)
                    d = dp[mask][j] + _dist(pos[j], pos[k])
                    if d < dp[nm][k]:
                        dp[nm][k] = d
                        par[nm][k] = j
        best_j, best_d = 0, INF
        for j in range(n):
            d = dp[full][j] + (_dist(pos[j], goal_pos) if goal_pos is not None else 0.0)
            if d < best_d:
                best_j, best_d = j, d
        seq = []
        mask, j = full, best_j
        while j != -1:
            seq.append(j)
            pj = par[mask][j]
            mask ^= (1 << j)
            j = pj
        seq.reverse()
        return [cps[i] for i in seq]
    # nearest neighbour for big sets
    rest = list(cps)
    order = []
    cur = start
    while rest:
        nxt = min(rest, key=lambda c: _dist(c['position'], cur))
        rest.remove(nxt)
        order.append(nxt)
        cur = nxt['position']
    return order


def _prune(start, order, goal_pos, with_goal_bonus, limit):
    """Drop checkpoints one by one while that raises the score (or fixes the time limit)."""
    skipped = []
    while order:
        cur_t = _est_time(start, order, goal_pos)
        cur_s = _score(len(order), cur_t, with_goal_bonus, limit)
        best = None
        for i in range(len(order)):
            sub = order[:i] + order[i + 1:]
            t = _est_time(start, sub, goal_pos)
            sc = _score(len(sub), t, with_goal_bonus, limit)
            if best is None or (sc, -t) > (best[0], -best[1]):
                best = (sc, t, i)
        improves = best[0] > cur_s + 0.5 or (cur_s == 0.0 and best[1] < cur_t)
        if not improves:
            break
        skipped.append(order[best[2]]['checkpoint_id'])
        order = order[:best[2]] + order[best[2] + 1:]
    return order, skipped


def plan_route(checkpoints, goal_id, start):
    """Return (route, skipped, with_goal).

    The judge gives the goal bonus (+30) only when EVERY checkpoint was passed
    ("goal: checkpoints not passed: ..." in result.json). So there are two plans:
      A) visit every checkpoint, goal last  -> (10*N + 30) points, only if it fits the time limit
      B) skip the ones that do not pay off, no goal bonus -> goal is an ordinary checkpoint
    The better estimated score wins."""
    cps = list(checkpoints)
    if not cps:
        return [], [], False
    goal_cp = None
    for cp in cps:
        if goal_id is not None and cp['checkpoint_id'] == goal_id:
            goal_cp = cp
    plan_a = None
    if goal_cp is not None:
        rest = [c for c in cps if c is not goal_cp]
        goal_pos = goal_cp['position']
        order_a = _best_order(start, rest, goal_pos)
        t_a = _est_time(start, order_a, goal_pos)
        s_a = _score(len(cps) - 1, t_a, True, PLAN_LIMIT_ALL)
        plan_a = (s_a, order_a + [goal_cp], t_a)
    order_b = _best_order(start, cps, None)
    order_b, skipped_b = _prune(start, order_b, None, False, PLAN_LIMIT)
    order_b = _best_order(start, order_b, None)
    t_b = _est_time(start, order_b, None)
    s_b = _score(len(order_b), t_b, False, PLAN_LIMIT)
    if plan_a is not None and plan_a[0] >= s_b:
        rospy.loginfo('plan A (all checkpoints + goal): score=%.1f est=%.1f  vs plan B score=%.1f est=%.1f',
                      plan_a[0], plan_a[2], s_b, t_b)
        return plan_a[1], [], True
    if plan_a is not None:
        rospy.loginfo('plan B (skip some, no goal): score=%.1f est=%.1f  vs plan A score=%.1f est=%.1f',
                      s_b, t_b, plan_a[0], plan_a[2])
    return order_b, skipped_b, False


def should_skip(elapsed, cur_pos, cp_pos, goal_pos):
    """Run-time guard: skip a checkpoint if we could not finish in time."""
    if elapsed is None:
        return False
    need = _hop(cur_pos, cp_pos) + SETTLE
    if goal_pos is not None:
        need += _hop(cp_pos, goal_pos) + GOAL_SETTLE
    return elapsed + need > GUARD_LIMIT


def parse_checkpoints(message):
    """Return (list of target dicts, goalpoint_id). Raises on bad data."""
    payload = json.loads(message)
    goal_id = payload.get('goalpoint_id')
    result = []
    for cp in payload.get('checkpoints', []):
        att = cp.get('attitude', {})
        quat = att.get('quaternion', [0.0, 0.0, 0.0, 1.0])
        result.append({
            'checkpoint_id': int(cp['checkpoint_id']),
            'position': [float(v) for v in cp['position'][:3]],
            'quaternion': [float(v) for v in quat[:4]],
        })
    return result, (int(goal_id) if goal_id is not None else None)


class UserTemplate:

    def __init__(self):
        rospy.init_node('krpc_user')

        # Action client
        self.__command = actionlib.SimpleActionClient('/ctl/command', CtlCommandAction)
        self.__checkpoint_client = actionlib.SimpleActionClient('/event/checkpoint_passed', CheckpointPassedAction)

        # Service client
        self.__mission_start = rospy.ServiceProxy('/event/mission_start', MissionStart)
        self.__mission_finished = rospy.ServiceProxy('/event/mission_finished', MissionFinished)
        self.__checkpoints = rospy.ServiceProxy('/mission/get_checkpoints', CheckPoints)

        # Subscriber
        self.__subscriber_start = rospy.Subscriber('/ib2_user/start', UserLogic, self.__callback_start)  # DO NOT REMOVE THIS LINE
        self.__subscriber_nav = rospy.Subscriber('/sensor_fusion/navigation', Navigation, self.__callback_nav)
        self.__current_position = None
        self.__current_orientation = None
        self.__last_stamp = None

        # Service server
        self.__stop_processing_server = rospy.Service('/ib2_user/stop', StopProcessingUserNode, self.__stop_processing)  # DO NOT REMOVE THIS LINE

        self.__started = False
        self.__action_active = False
        rospy.set_param('/ib2_user/ready', True)  # DO NOT REMOVE THIS LINE

    def __notify_mission_start(self):
        rospy.wait_for_service('/event/mission_start')
        res = self.__mission_start()
        rospy.loginfo('mission_start result: success=%s message=%s', res.success, res.message)
        return res.success

    def __notify_mission_finished(self):
        rospy.wait_for_service('/event/mission_finished')
        res = self.__mission_finished()
        rospy.loginfo('mission_finished result: success=%s message=%s', res.success, res.message)
        return res.success

    def __notify_checkpoint_passed(self, checkpoint_id):
        rospy.loginfo('notify checkpoint_id=%d passed', checkpoint_id)
        self.__checkpoint_client.wait_for_server()
        goal = CheckpointPassedGoal()
        goal.checkpoint_id = checkpoint_id
        self.__checkpoint_client.send_goal(goal)
        while not rospy.is_shutdown():
            if self.__checkpoint_client.wait_for_result(rospy.Duration(0.2)):
                result = self.__checkpoint_client.get_result()
                rospy.loginfo('checkpoint_passed result: %s', result)
                return result.success
        if rospy.is_shutdown():
            self.__checkpoint_client.cancel_goal()
            rospy.logwarn('checkpoint_passed action cancelled by node shutdown')
        return False

    def __load_checkpoints(self):
        rospy.wait_for_service('/mission/get_checkpoints')
        res = self.__checkpoints()
        rospy.loginfo('get_checkpoints result: success=%s message=%s', res.success, res.message)
        if not res.success:
            return [], None
        try:
            return parse_checkpoints(res.message)
        except Exception as e:
            rospy.logerr('Failed to parse checkpoints payload: %s', e)
            return [], None

    def __move_to_checkpoint(self, checkpoint_target):
        position = checkpoint_target['position']
        quaternion = checkpoint_target['quaternion']
        rospy.loginfo(
            'moving to checkpoint_id=%d: x=%.3f y=%.3f z=%.3f qx=%.4f qy=%.4f qz=%.4f qw=%.4f',
            checkpoint_target['checkpoint_id'],
            position[0], position[1], position[2],
            quaternion[0], quaternion[1], quaternion[2], quaternion[3]
        )

        target = PoseStamped()
        target.pose.position.x = position[0]
        target.pose.position.y = position[1]
        target.pose.position.z = position[2]
        target.pose.orientation.x = quaternion[0]
        target.pose.orientation.y = quaternion[1]
        target.pose.orientation.z = quaternion[2]
        target.pose.orientation.w = quaternion[3]

        command_type = CtlStatusType(type=CtlStatusType.MOVE_TO_ABSOLUTE_TARGET)
        goal = CtlCommandGoal(target=target, type=command_type)
        self.__action_active = True
        try:
            self.__command.send_goal(goal)
            _t0 = rospy.get_time()
            while not rospy.is_shutdown():
                _e = self.__pose_error(checkpoint_target)
                if _e is not None:
                    rospy.loginfo('MV cp=%d t=%.1f pe=%.3f ae=%.1f',
                                  checkpoint_target['checkpoint_id'], rospy.get_time() - _t0, _e[0], _e[1])
                if self.__command.wait_for_result(rospy.Duration(0.2)):
                    result = self.__command.get_result()
                    rospy.loginfo('move command result: %s', result)
                    try:
                        self.__last_stamp = result.stamp.to_sec()
                    except Exception:
                        self.__last_stamp = None
                    return result.type == CtlCommandResult.TERMINATE_SUCCESS
            if rospy.is_shutdown():
                self.__command.cancel_goal()
                rospy.logwarn('move command cancelled by node shutdown')
            return False
        finally:
            self.__action_active = False

    def __pose_error(self, cp):
        if self.__current_position is None or self.__current_orientation is None:
            return None
        perr = _dist(self.__current_position, cp['position'])
        dot = abs(sum(a * b for a, b in zip(self.__current_orientation, cp['quaternion'])))
        ang = math.degrees(2.0 * math.acos(min(1.0, dot)))
        return perr, ang

    def __refine(self, cp, allowed):
        """If the robot is clearly off the target pose (a Fail would also cost the goal),
        wait a little, then send the move again. Returns True when a move was re-sent."""
        err = self.__pose_error(cp)
        if err is None or err[1] + 114.6 * err[0] <= REFINE_LIMIT:
            return False
        rospy.logwarn('REFINE cp=%d pos_err=%.3f att_err=%.1f allowed=%s',
                      cp['checkpoint_id'], err[0], err[1], allowed)
        for _ in range(int(REFINE_WAIT / 0.2)):
            if rospy.is_shutdown():
                return False
            rospy.sleep(0.2)
            err = self.__pose_error(cp)
            if err is not None and err[1] + 114.6 * err[0] <= REFINE_LIMIT:
                rospy.loginfo('REFINE cp=%d settled by waiting', cp['checkpoint_id'])
                return False
        if not allowed:
            rospy.logwarn('REFINE cp=%d skipped (no time)', cp['checkpoint_id'])
            return False
        self.__move_to_checkpoint(cp)
        rospy.sleep(SETTLE)
        return True

    def __log_error(self, cp):
        try:
            if self.__current_position is None or self.__current_orientation is None:
                return
            perr = _dist(self.__current_position, cp['position'])
            dot = abs(sum(a * b for a, b in zip(self.__current_orientation, cp['quaternion'])))
            ang = math.degrees(2.0 * math.acos(min(1.0, dot)))
            rospy.loginfo('CHECK cp=%d pos_err=%.3f m att_err=%.1f deg', cp['checkpoint_id'], perr, ang)
        except Exception as e:
            rospy.logwarn('log_error failed: %s', e)

    def __run_user_flow(self, msg):
        try:
            if not self.__notify_mission_start():
                return

            # USER EDIT AREA: route planning and game-specific flow.
            # **************************************************
            checkpoints, goal_id = self.__load_checkpoints()
            if not checkpoints:
                rospy.logerr('no checkpoints, finishing')
            start = self.__current_position if self.__current_position else (
                checkpoints[0]['position'] if checkpoints else [0.0, 0.0, 0.0])
            route, skipped, with_goal = plan_route(checkpoints, goal_id, start)
            goal_pos = route[-1]['position'] if with_goal else None
            rospy.loginfo('goal_id=%s with_goal=%s route=%s planned_skip=%s est_time=%.1f',
                          goal_id, with_goal, [c['checkpoint_id'] for c in route], skipped,
                          _est_time(start, route[:-1] if with_goal else route, goal_pos))

            cur_pos = start
            elapsed = 0.0
            for idx, cp in enumerate(route):
                if rospy.is_shutdown():
                    return
                is_goal = with_goal and (cp is route[-1])
                if not is_goal and should_skip(elapsed, cur_pos, cp['position'], goal_pos):
                    rospy.logwarn('GUARD skip checkpoint %d (elapsed=%.1f)', cp['checkpoint_id'], elapsed)
                    continue
                reached = self.__move_to_checkpoint(cp)
                tries = 1
                while not reached and tries < MOVE_TRIES and not rospy.is_shutdown():
                    rospy.logwarn('move to checkpoint %d failed (try %d), retrying', cp['checkpoint_id'], tries)
                    reached = self.__move_to_checkpoint(cp)
                    tries += 1
                if not reached:
                    rospy.logwarn('move to checkpoint %d failed after %d tries, notifying anyway', cp['checkpoint_id'], tries)
                if self.__last_stamp is not None and self.__last_stamp > 0:
                    elapsed = self.__last_stamp - MISSION_T0
                cur_pos = cp['position']
                rospy.sleep(GOAL_SETTLE if is_goal else SETTLE)
                rest = route[idx + 1:]
                if not rest:
                    rem = 0.0
                elif with_goal:
                    rem = _est_time(cp['position'], rest[:-1], rest[-1]['position']) - START_EXTRA
                else:
                    rem = _est_time(cp['position'], rest, None) - START_EXTRA
                if self.__refine(cp, elapsed + REFINE_COST + rem <= GUARD_LIMIT):
                    if self.__last_stamp is not None and self.__last_stamp > 0:
                        elapsed = self.__last_stamp - MISSION_T0
                self.__log_error(cp)
                ok = self.__notify_checkpoint_passed(cp['checkpoint_id'])
                rospy.loginfo('checkpoint %d pass notified: %s elapsed=%.1f',
                              cp['checkpoint_id'], ok, elapsed)
            # **************************************************

            if not rospy.is_shutdown():
                self.__notify_mission_finished()
        except Exception as exc:
            rospy.logerr('user flow failed: %s', exc)

    def __callback_start(self, msg):
        if self.__started:
            rospy.logwarn('start callback is already processed')
            return
        self.__started = True

        rospy.loginfo('start %s', msg)
        self.__run_user_flow(msg)

    def __callback_nav(self, data):
        p = data.pose.pose.position
        self.__current_position = [p.x, p.y, p.z]
        o = data.pose.pose.orientation
        self.__current_orientation = [o.x, o.y, o.z, o.w]

    def __stop_processing(self, request):
        try:
            if self.__action_active:
                self.__command.cancel_goal()
            rospy.loginfo('finish processing')
            return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.SUCCESS)
        except Exception as exc:
            rospy.logerr('stop processing failed: %s', exc)
            return StopProcessingUserNodeResponse(result=StopProcessingUserNodeResponse.ERROR)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    UserTemplate().run()
