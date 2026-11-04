"""Arena navigation/telemetry substitute; production ROS and PTY own commands.

Physical pose and folding feedback come from immutable engine observations.
Navigation starts on an accepted request and finishes from the simulated
controller result; no terminal arrival or folding completion is injected.
Ordinary navigation uses the UC goal/status/cancel topics. The tunnel action
server below is a simulation substitute, not evidence of production Nav2 wiring.
"""
import copy
import json
import math
import threading
import time

class InterfaceBridge:
    DEFAULTS = dict(nav_goal='/goal_pose', nav_cancel='/nav_cancel', nav_status='/nav_status',
        cmd_vel='/cmd_vel', chassis_mode='/move_mode', odom='/odom', fold_cmd='/gimbal_fold_cmd',
        data_source='/sentry/data_source', attack_cmd='/sentry/attack_cmd')

    def __init__(self, node, config, fmap, serial, record=None):
        from geometry_msgs.msg import PoseStamped, Twist
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Empty, Int32, String, Int32MultiArray
        self.node, self.fmap, self.serial = node, fmap, serial
        self.t = {k: config.get('topics',{}).get('sim',{}).get(k) or v for k,v in self.DEFAULTS.items()}
        self.lock = threading.Lock()
        self.navigation_lock = threading.Lock()
        self.goal = self.pending_goal = None
        self.route = None
        self.route_handle = None
        self.route_future = None
        self.revision = 0
        self.trace = None
        self.attack = [0,0,0,0]
        self.attack_time = -1e9
        self.goal_events, self.trace_events, self.rx_count = [], [], {}
        self.route_events = []
        self.time_s = 0.
        self.record = record or (lambda *args: None)
        self.pub_odom = node.create_publisher(Odometry, self.t['odom'], 10)
        self.pub_nav = node.create_publisher(Int32, self.t['nav_status'], 10)
        self.pub_velocity = node.create_publisher(Twist, self.t['cmd_vel'], 10)
        self.pub_source = node.create_publisher(String, self.t['data_source'], 10)
        node.create_subscription(PoseStamped, self.t['nav_goal'], self.on_goal, 100)
        node.create_subscription(Empty, self.t['nav_cancel'], self.on_cancel, 100)
        node.create_subscription(String, '/sentry/decision_trace', self.on_trace, 100)
        node.create_subscription(Int32MultiArray, self.t['attack_cmd'], self.on_attack, 10)
        from nav2_msgs.action import NavigateThroughPoses
        from rclpy.action import ActionServer, GoalResponse, CancelResponse
        from run_tunnel_suite import rotate_geometry
        tunnels=rotate_geometry(config,serial.team)
        known_routes=[(tuple(t['center']),tuple(a['p']))
            for t in tunnels if t.get('center') is not None for a in t['align']]
        known_exits=[tuple(a['p']) for t in tunnels for a in t['align']]
        def accept(request):
            points=[(p.pose.position.x,p.pose.position.y) for p in request.poses]
            # This simulator substitutes Nav2 only for the configured collinear
            # tunnel route; an arbitrary path must go to a real Nav2 server.
            configured=(len(points)==1 and any(math.dist(points[0],p)<.01 for p in known_exits)) or (
                len(points)==2 and any(all(math.dist(p,q)<.01 for p,q in zip(points,route)) for route in known_routes))
            valid=(configured and all(math.isfinite(v) for p in points for v in p)
                and all(p.header.frame_id=='map' for p in request.poses)
                )
            return GoalResponse.ACCEPT if valid else GoalResponse.REJECT
        def cancel(handle):
            with self.lock:
                if (self.route_handle is handle and self.route_future is not None
                        and not self.route_future.done()):
                    self.goal=self.pending_goal=None;self.revision+=1
                    if self.route_future and not self.route_future.done():self.route_future.set_result(False)
                    self.pub_velocity.publish(Twist())
            return CancelResponse.ACCEPT
        async def execute(handle):
            from rclpy.task import Future
            result=NavigateThroughPoses.Result()
            if handle.is_cancel_requested:
                handle.canceled()
                return result
            points=[(p.pose.position.x,p.pose.position.y) for p in handle.request.poses]
            with self.lock:
                if self.route_future and not self.route_future.done():self.route_future.set_result(False)
                self.route_handle=handle;self.route_future=Future();done=self.route_future
                self.route=points;self.pending_goal=points[-1];self.revision+=1
                event=dict(time_s=self.time_s,poses=[list(p) for p in points],terminal_goal=list(points[-1]),
                    intermediate_behavior='pass_through',action='NavigateThroughPoses')
                self.route_events.append(event);self.record('nav_route',event)
            succeeded=await done
            if handle.is_cancel_requested:handle.canceled()
            elif succeeded:handle.succeed()
            else:handle.abort();result.error_code=1;result.error_msg='sim navigation failed or superseded'
            with self.lock:
                if self.route_handle is handle:
                    self.route=self.route_handle=self.route_future=None
            return result
        self.route_server=ActionServer(node,NavigateThroughPoses,'/navigate_through_poses',
            execute_callback=execute,goal_callback=accept,cancel_callback=cancel)

    def bump(self,key):
        self.rx_count[key]=self.rx_count.get(key,0)+1

    def on_goal(self,msg):
        from std_msgs.msg import Int32
        with self.lock:
            self.bump('nav_goal')
            item=dict(time_s=self.time_s,xy=[msg.pose.position.x,msg.pose.position.y],frame_id=msg.header.frame_id,
                      stamp_sec=msg.header.stamp.sec,stamp_nanosec=msg.header.stamp.nanosec)
            self.goal_events.append(item); self.record('goal',item)
            if self.route_future is not None and not self.route_future.done():
                self.route_future.set_result(False)
            # A replacement ordinary request can arrive before the old action's
            # asynchronous UUID cancellation. Detach that action before install
            # so its eventual cancellation or completion cannot clear this goal.
            self.route=self.route_handle=self.route_future=None
            # Ordinary navigation uses the UC goal/status topics. The internal
            # Arena obstacle route never becomes another BBT destination.
            self.pending_goal=(msg.pose.position.x,msg.pose.position.y)
            self.revision+=1
            self.pub_nav.publish(Int32(data=1))

    def on_cancel(self,_):
        from geometry_msgs.msg import Twist
        with self.lock:
            # Nav2 cancellation is scoped by action UUID. A delayed Empty from
            # the old single-point entrance must not cancel a newly accepted route.
            if self.route_future is not None and not self.route_future.done():
                self.record('single_goal_cancel_ignored_during_route',None)
                return
            self.bump('nav_cancel'); self.revision += 1
            self.goal=self.pending_goal=None
            self.pub_velocity.publish(Twist())
            self.record('cancel',None)
            if self.route_future and not self.route_future.done():self.route_future.set_result(False)

    def on_trace(self,msg):
        try: trace=json.loads(msg.data)
        except (ValueError,TypeError): return
        with self.lock:
            self.bump('decision_trace'); self.trace=trace
            self.trace_events.append(dict(time_s=self.time_s,trace=trace))
            self.record('decision_trace',trace)

    def on_attack(self,msg):
        if len(msg.data)>=4:
            with self.lock:
                self.attack=list(msg.data[:4]); self.attack_time=time.monotonic()

    def announce_source(self):
        from std_msgs.msg import String
        self.pub_source.publish(String(data='sim'))

    def orders(self):
        with self.lock:
            # Apply only goals received after the latest cancellation. Topic
            # ordering cannot establish that an Empty belongs to a replacement.
            if self.pending_goal is not None:
                self.goal=self.pending_goal; self.pending_goal=None
            return self.goal,self.revision,copy.deepcopy(self.trace)

    def publish_observation(self,sentry,obs,frame,fold_report):
        from nav_msgs.msg import Odometry
        x,y=self.fmap.to_field_xy(sentry['x'],sentry['y'])
        yaw=self.fmap.to_field_yaw(sentry['heading'])
        vx,vy=self.fmap.to_field_vec(sentry['vx'],sentry['vy'])
        msg=Odometry();msg.header.stamp=self.node.get_clock().now().to_msg()
        msg.header.frame_id='map';msg.child_frame_id='base_link'
        msg.pose.pose.position.x=x;msg.pose.pose.position.y=y
        msg.pose.pose.orientation.z=math.sin(yaw/2);msg.pose.pose.orientation.w=math.cos(yaw/2)
        msg.twist.twist.linear.x=vx;msg.twist.twist.linear.y=vy
        self.pub_odom.publish(msg)
        self.serial.pump();self.serial.referee(obs,sentry,frame,self.fmap)
        self.serial.chassis(sentry,self.fmap,fold_report.update(sentry))
        return (x,y)

    def navigation(self,nav,monitor,robot,arena):
        from std_msgs.msg import Int32
        from geometry_msgs.msg import Twist
        goal,revision,trace=self.orders()
        with self.lock:
            if revision != self.revision:
                return self.goal,copy.deepcopy(self.trace)
        pos=self.fmap.to_field_xy(robot['x'],robot['y'])
        straight=goal is not None and monitor.crossing(*pos,*goal) is not None
        corridor=next((c for c in monitor.corridors if goal is not None and
                       any(math.dist(goal,p)<.01 for p in c[1:4])),None)
        with self.navigation_lock:
            vx,vy=nav.update(goal,pos,robot,arena,straight=straight,
                precise=monitor.is_alignment(goal),revision=revision,
                corridor=corridor)
        with self.lock:
            # Computing a velocity can overlap another ROS callback. Its result
            # belongs exclusively to the goal revision sampled before update.
            if revision != self.revision:
                return self.goal,copy.deepcopy(self.trace)
            if self.route_future and not self.route_future.done():
                # Intermediate centre is on the configured corridor axis; the
                # single controller target is always the final exit, so there
                # is no centre success, cancellation or deceleration request.
                if nav.state in (2,3):
                    self.record('nav_route_result',dict(state=nav.state,goal=goal,position=list(pos)))
                    self.route_future.set_result(nav.state==2)
            self.pub_nav.publish(Int32(data=nav.state))
            if goal is not None:
                msg=Twist();msg.linear.x,msg.linear.y=self.fmap.to_field_vec(vx,vy)
                self.pub_velocity.publish(msg)
        return goal,trace


def serial_chassis_action(serial,sentry,fmap):
    """UC chassis contract, including model speed reference and explicit yaw."""
    cvx,cvy,w=serial.velocity();ax,ay=fmap.to_arena_vec(cvx,cvy)
    speed=float(sentry['speed_reference_mps'])*(.5 if sentry['crouched'] else 1.)
    if not math.isfinite(speed) or speed<=0:raise ValueError('invalid chassis speed reference')
    if serial.mode==0:ax,ay,w=0.,0.,0.
    magnitude=math.hypot(ax,ay)
    move=[ax/magnitude*min(1.,magnitude/speed),ay/magnitude*min(1.,magnitude/speed)] if magnitude>1e-6 else [0.,0.]
    action=dict(move=move,yaw_rate=max(-12.,min(12.,-w)),crouch=serial.fold_cmd==1,
                aim=float(sentry['aim']),pitch=.04,fire=False)
    action.update(serial.purchase_action(sentry))
    return action


def replay_decision(trace,team,goal,attack=(0,0,0,0)):
    """Present actual BBT trace/ROS outputs in the existing Arena HUD schema.

    Raw BBT state is retained verbatim in bbt_trace. The HUD's UC compatibility
    fields are display-only and never sent back to ROS or used for acceptance.
    """
    obs=[0.]*14;obs[10]=1.;obs[13]=trace.get('current_region',-1)
    kind,tid,fire,free=attack
    return dict(seq=trace['seq'],source=0,rl_reason=f'BBT 保底 · 阶段 {trace.get("phase",0)}',
        safe=dict(point=trace.get('target_point',-1),tunnel=trace.get('tunnel',-1),
            type=kind,id=tid,fire=bool(fire),free=bool(free),buy_ammo=False,buy_hp=False),
        obs=obs,team=team,test_policy=False,target_xy=list(goal) if goal else None,bbt_trace=copy.deepcopy(trace))
