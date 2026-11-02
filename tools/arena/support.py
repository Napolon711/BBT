"""Arena adapters reused from 27BT-UC-simside (2026-10-31).
Geometry/navigation/replay only; decisions stay in the production BBT process.
"""
import math
import json
from pathlib import Path

try:
    from .navigation_constraints import bumps_clearance, sentry_routing_map, point_segment_distance
    from .nav_plan import remaining_route
except ImportError:  # Direct execution of adapter tools and offline tests.
    from navigation_constraints import bumps_clearance, sentry_routing_map, point_segment_distance
    from nav_plan import remaining_route
FIELD_W=28.0
FIELD_H=15.0
sdk=None

class FrameMap:
    """Fixed Arena/referee transform. Team affects IDs and tactical geometry only."""
    def __init__(self, own_team):
        if own_team not in ('red', 'blue'):
            raise ValueError(own_team)
        self.own_team = own_team
        self.enemy_team = 'blue' if own_team == 'red' else 'red'
    def to_field_xy(self, x, y): return (x, FIELD_H-y)
    def to_field_yaw(self, yaw): return -yaw
    def to_field_vec(self, vx, vy): return (vx, -vy)
    to_arena_xy = to_field_xy
    to_arena_yaw = to_field_yaw
    to_arena_vec = to_field_vec

class TunnelMonitor:
    """统计车**真的**进了哪条隧道，供验收用。

    为什么需要：`/current_region` 由"最近已标注点"判定，车绕过隧道北端时也会
    因为离某个标为 mid 的点更近而翻区域——**区域翻转不等于过洞**。上一轮就是
    把区域翻转误当成"过洞成功"（实测 1800 拍采样 0 次进洞）。所以这里另立一个
    纯几何判据：车心是否落在 decision.yaml 的隧道矩形内，只做统计不改任何输出。

    参数 tunnels — decision.yaml 的 tunnels 列表（每项含 name/corners）。
    副作用：内部维护 inside（当前是否在洞内）与 counts（各洞进入次数）。
    """

    def __init__(self, tunnels):
        self.tunnels = []
        self.alignments = []
        self.corridors = []
        for t in tunnels or []:
            self.alignments.extend(tuple(a["p"]) for a in t.get("align", []))
            center=t.get('center');align=t.get('align',[])
            if center is not None and len(align)==2:
                a,b=(tuple(item['p']) for item in align)
                length=math.dist(a,b)
                if length>1e-6:
                    self.alignments.append(tuple(center))
                    self.corridors.append((t.get('name','?'),a,tuple(center),b,
                                           tuple((b[i]-a[i])/length for i in range(2)),length))
            c = t.get("corners") or []
            if len(c) < 3:
                continue
            xs = [p[0] for p in c]
            ys = [p[1] for p in c]
            self.tunnels.append((t.get("name", "?"), min(xs), max(xs), min(ys), max(ys)))
        self.inside = None          # 当前所在洞名，None = 不在洞内
        self.counts = {}            # 洞名 → 进入次数
        self.inside_ticks = 0       # 洞内累计拍数

    def is_alignment(self, goal):
        return goal is not None and any(math.hypot(goal[0]-a[0],goal[1]-a[1]) < .01 for a in self.alignments)

    def crossing(self, px, py, gx, gy, tol=0.35, lat=0.20, reach=6.0):
        """判断"从当前位置去目标点"是否要穿某条隧道，要穿则返回洞名。

        为什么按几何判据而不是折叠状态：决策层**故意**在中止过洞时不展开
        （bt_nodes.cpp `onHalted`：「退出节点不等于离开洞口，不能自动展开」），
        所以"已折叠"不等于"正在过洞"。若把走直线绑在折叠状态上，车一旦被留在
        折叠态，之后所有导航都变直线并撞墙卡死（实测卡在场地南缘 150 s）。

        判据（三个条件同时满足才算过洞）：
          1. **两端分居洞的两侧**：车与目标沿洞轴分别在矩形两端之外
             （如南北走向的洞，一个在 y0 以南、另一个在 y1 以北）。
             对正点正好落在矩形边上，所以两端各放宽 tol；
          2. **两端都对准洞口**：车与目标的横向坐标都落在洞口宽度内
             （放宽 lat=0.20 m）。洞口只有 0.80~1.0 m 宽、车半宽 0.293 m，
             横向可容忍的偏差只有约 0.2 m，再斜就会撞洞壁；
          3. **就近**：两端都在洞口 reach 米内，避免在场地另一头误触发。

        参数：px,py — 车场地系位置；gx,gy — 目标点场地系位置；
              tol — 端点落在矩形边上的容差 m；lat — 横向对准容差 m；
              reach — 就近判定的半径 m。
        返回：要穿的隧道名；不需要穿洞时返回 None。
        副作用：无。
        """
        # Production publishes three separate goals: entry, configured centre,
        # then exit. Only exact corridor waypoints authorize a straight leg;
        # being folded by itself never changes ordinary navigation into a line.
        for name,a,center,b,axis,length in self.corridors:
            lateral=(-axis[1],axis[0])
            along=lambda p:sum((p[i]-a[i])*axis[i] for i in range(2))
            cross=lambda p:abs(sum((p[i]-a[i])*lateral[i] for i in range(2)))
            current=(px,py);goal=(gx,gy)
            p=along(current);c=along(center)
            if cross(current)>lat or not -tol<=p<=length+tol:
                continue
            center_goal=math.dist(goal,center)<.01
            exit_goal=math.dist(goal,b)<.01 and p>=c-tol
            reverse_exit=math.dist(goal,a)<.01 and p<=c+tol
            if center_goal or exit_goal or reverse_exit:
                return name
        for name, x0, x1, y0, y1 in self.tunnels:
            if not (x0 - reach <= px <= x1 + reach and y0 - reach <= py <= y1 + reach
                    and x0 - reach <= gx <= x1 + reach and y0 - reach <= gy <= y1 + reach):
                continue
            if (y1 - y0) >= (x1 - x0):
                # 洞轴沿 y（南北走向）：看 y 是否分居两端，x 是否都对准
                if not ((py <= y0 + tol and gy >= y1 - tol)
                        or (py >= y1 - tol and gy <= y0 + tol)):
                    continue
                if not (x0 - lat <= px <= x1 + lat and x0 - lat <= gx <= x1 + lat):
                    continue
            else:
                # 洞轴沿 x（东西走向）：看 x 是否分居两端，y 是否都对准
                if not ((px <= x0 + tol and gx >= x1 - tol)
                        or (px >= x1 - tol and gx <= x0 + tol)):
                    continue
                if not (y0 - lat <= py <= y1 + lat and y0 - lat <= gy <= y1 + lat):
                    continue
            return name
        return None

    def update(self, fx, fy):
        """按车心位置更新统计。参数 fx,fy — 场地系位置 m。副作用：更新内部计数。"""
        now_in = None
        for name, x0, x1, y0, y1 in self.tunnels:
            if x0 <= fx <= x1 and y0 <= fy <= y1:
                now_in = name
                break
        if now_in is not None:
            self.inside_ticks += 1
        if now_in != self.inside:
            if now_in is not None:
                self.counts[now_in] = self.counts.get(now_in, 0) + 1
            self.inside = now_in

    def summary(self):
        """返回一行人类可读的过洞统计。副作用：无。"""
        if not self.tunnels:
            return "过洞统计：decision.yaml 里没有隧道矩形"
        if not self.counts:
            return f"过洞统计：**一次都没进洞**（洞内拍数 {self.inside_ticks}）"
        parts = ", ".join(f"{k}×{v}" for k, v in sorted(self.counts.items()))
        return f"过洞统计：{parts}（洞内累计 {self.inside_ticks} 拍）"

class NavController:
    """把决策层的"目标点"变成 Arena 的"速度方向"，并回报导航状态。

    为什么需要它：决策层下发的是**目标点**（实车上由 Nav2 执行），Arena 只吃
    `move` 速度方向。所以桥必须扮演导航控制器这一层。

    平时为什么不能直着开：实车上这件事由 Nav2 的代价地图 + 全局规划解决，所以桥用
    Arena 自己的 `sdk.navigate()`——它按膨胀后的墙角逐点搜一条无碰撞折线，每拍返回
    "朝下一个航点"的方向，等价于一个路径跟随器。

    **过洞段必须例外（straight=True）**：`sdk.navigate()` 把障碍按 `半径 + 0.08 m`
    膨胀，而公路隧道洞口净宽只有 0.80 m、哨兵半径 0.389 m，膨胀后两侧墙在
    x∈[18.61,18.75] 处重叠 —— 隧道对寻路器是**完全封死**的，它会绕到东侧一大圈、
    从路面北端外穿过区域边界（实测 1800 拍采样 0 次进洞）。所以折叠期间改走
    "朝目标点的直线"：两个对正点都在洞口中线上，直线即洞轴，能真正穿洞。
    实测蹲下直线穿洞可到场地 y=10.53（南侧），不蹲则卡在洞顶北缘 y=13.16。

    参数：
      fmap — FrameMap
      arrive_m — 判定"到达"的距离阈值，m
      max_speed — 车速上限，m/s（Arena 哨兵 speed=4.0）
    副作用：内部维护 state（0 idle / 1 running / 2 success / 3 failure）。
    """

    def __init__(self, fmap, arrive_m=0.08, max_speed=2.9):
        self.fmap = fmap
        self.arrive_m = arrive_m
        self.max_speed = max_speed
        self.state = 0
        self.goal_arena = None
        self.straight = False
        self.goal_key = None
        self.precise = False
        self.routing_source = None
        self.routing_arena = None
        self.plan_arena = []
        self.crossing_axis_arena = None
        self.exit_clear_arena = None
        self.exit_axis_arena = None
        self.corridor = None
        self.lateral_integral=0.
        self.longitudinal_integral=0.
        self.last_failure = None
        self.fixed_path = None
        self.fixed_path_index = 1
        self.fixed_last_position = None

    def prepare(self, arena, robot):
        if self.routing_source is not arena:
            self.routing_source = arena
            self.routing_arena = sentry_routing_map(arena)
            prepare=getattr(sdk,"_prepare_chapter4",None)
            if prepare is not None:
                # Warm both envelopes before live ROS freshness timers start.
                prepare(self.routing_arena,robot["team"],True)
                prepare(self.routing_arena,robot["team"],False)

    def plan(self, goal_field, robot, arena):
        """Return the complete ordinary route in Arena coordinates, or [].

        Planning is separate from execution so the production path gate sees
        exactly the polyline subsequently passed to FollowPath. The axial
        connector established by a completed crossing remains part of the plan.
        """
        self.prepare(arena,robot)
        if goal_field is None:
            return []
        goal=self.fmap.to_arena_xy(*goal_field)
        radius=max(.47,float(robot.get('radius',.42)))
        if bumps_clearance(arena,goal,radius=radius)<=.01:
            return []
        start=(robot['x'],robot['y'])
        ordinary=None
        if self.exit_clear_arena is not None and math.dist(start,self.exit_clear_arena)<.15:
            ordinary=remaining_route(sdk,robot,goal,self.routing_arena,radius)
            if ordinary:
                self.exit_clear_arena=None
        if self.exit_clear_arena is None:
            return ordinary or remaining_route(sdk,robot,goal,self.routing_arena,radius)
        initial=self.exit_clear_arena
        for extension in (0.,.25,.5,.75,1.):
            direction=self.exit_axis_arena or (0.,0.)
            connector=tuple(initial[i]+extension*direction[i] for i in range(2))
            boundary_margin=float(robot.get('radius',.42))+.02
            if not boundary_margin<connector[0]<28-boundary_margin or not boundary_margin<connector[1]<15-boundary_margin:
                continue
            first=remaining_route(sdk,robot,connector,self.routing_arena,radius)
            planning_start=dict(robot,x=connector[0],y=connector[1])
            second=remaining_route(sdk,planning_start,goal,self.routing_arena,radius)
            if first and second:
                self.exit_clear_arena=connector
                return first+second[1:]
        return []

    def _fixed_segment_clear(self, a, b, robot, arena, radius):
        if bumps_clearance(arena,a,b,radius)<=0:
            return False
        prepare=getattr(sdk,'_prepare_chapter4',None)
        if prepare is None:
            # Empty offline maps require no graph. A real obstacle map without
            # a collision query cannot safely authorize fixed-path motion.
            return not arena.get('obstacles')
        _,_,clear,inside=prepare(self.routing_arena,robot['team'],robot.get('transform_progress',0.)>=.99)
        near_start=inside(a,True);near_goal=inside(b)
        ignore=({**near_start,**dict.fromkeys(near_goal)} if isinstance(near_start,dict)
                else near_start|near_goal)
        return clear(a,b,ignore,False)

    def update(self, goal_field, pose_field, robot, arena, straight=False,
               precise=False, revision=0, corridor=None, fixed_path=None):
        """Navigation goal owns its path until completion/cancel; never replan out of a tunnel.

        Alignment goals require <=2.5 cm position error and <=0.12 m/s actual speed.
        Success remains latched for that goal; a new ROS goal/cancel resets it.
        """
        if self.state!=3:
            self.last_failure = None
        key = (tuple(goal_field) if goal_field is not None else None, revision)
        if key != self.goal_key:
            self.goal_key = key
            self.state = 0
            self.last_failure = None
            self.straight = bool(straight)
            self.precise = bool(precise)
            self.corridor=corridor
            self.lateral_integral=0.
            self.longitudinal_integral=0.
            self.fixed_path=(tuple(self.fmap.to_arena_xy(*p) for p in fixed_path)
                             if fixed_path is not None else None)
            self.fixed_path_index=1
            self.fixed_last_position=None
            self.crossing_axis_arena = None
            if self.straight and goal_field is not None:
                goal=self.fmap.to_arena_xy(*goal_field)
                dx,dy=goal[0]-robot['x'],goal[1]-robot['y']
                length=math.hypot(dx,dy)
                if length>1e-6:self.crossing_axis_arena=(dx/length,dy/length)
            if self.straight and self.corridor is not None:
                _,a,_,b,field_axis,_=self.corridor
                arena_axis=self.fmap.to_arena_vec(*field_axis)
                direction=1. if sum((goal_field[i]-pose_field[i])*field_axis[i] for i in range(2))>=0 else -1.
                self.crossing_axis_arena=tuple(direction*x for x in arena_axis)
        if goal_field is None:
            self.state = 0
            self.goal_arena = None
            self.plan_arena = []
            self.straight = False
            return (0.0, 0.0)
        self.goal_arena = self.fmap.to_arena_xy(*goal_field)
        self.prepare(arena,robot)
        radius=max(.47,float(robot.get('radius',.42)))
        start=(robot["x"],robot["y"])
        self.plan_arena=(list(self.fixed_path) if self.fixed_path is not None
                         else [start,self.goal_arena])
        if self.fixed_path is not None:
            if self.state==3:
                return (0.,0.)
            if (len(self.fixed_path)<2 or
                    any(not all(math.isfinite(x) for x in p) for p in self.fixed_path) or
                    math.dist(self.fixed_path[-1],self.goal_arena)>.01):
                self.state=3;self.last_failure='invalid_fixed_path'
                return (0.,0.)
            if self.state==0 and math.dist(start,self.fixed_path[0])>.3:
                self.state=3;self.last_failure='fixed_path_start_deviation'
                return (0.,0.)
            if self.state==0 and any(not self._fixed_segment_clear(a,b,robot,arena,radius)
                    for a,b in zip(self.fixed_path,self.fixed_path[1:])):
                self.state=3;self.last_failure='fixed_path_blocked'
                return (0.,0.)
            while (self.fixed_path_index<len(self.fixed_path)-1 and
                    (math.dist(start,self.fixed_path[self.fixed_path_index])<=.08 or
                     (self.fixed_last_position is not None and
                      point_segment_distance(self.fixed_path[self.fixed_path_index],
                          self.fixed_last_position,start)<=.08))):
                self.fixed_path_index+=1
                self.longitudinal_integral=0.
                self.lateral_integral=0.
            previous=self.fixed_path[self.fixed_path_index-1]
            next_point=self.fixed_path[self.fixed_path_index]
            if point_segment_distance(start,previous,next_point)>.3:
                self.state=3;self.last_failure='fixed_path_tracking_deviation'
                return (0.,0.)
            self.fixed_last_position=start
        if bumps_clearance(arena,self.goal_arena,radius=radius)<=.01:
            self.state=3  # A discrete point never authorizes forbidden terrain.
            self.last_failure = 'goal_footprint_overlaps_bumps'
            return (0.,0.)
        if self.state == 2:
            return (0.0, 0.0)
        distance = math.dist(start,self.goal_arena)
        tolerance = 0.025 if self.precise else self.arrive_m
        measured_speed = math.hypot(robot['vx'],robot['vy'])
        cross_error=0.;axis=None
        if self.corridor is not None:
            _,origin,_,_,field_axis,_=self.corridor
            axis=self.fmap.to_arena_vec(*field_axis)
            origin=self.fmap.to_arena_xy(*origin)
            lateral=(-axis[1],axis[0])
            cross_error=sum((start[i]-origin[i])*lateral[i] for i in range(2))
        self.state = 1
        final_segment=self.fixed_path is None or self.fixed_path_index==len(self.fixed_path)-1
        if final_segment and distance <= tolerance and (axis is None or abs(cross_error)<=.02):
            if measured_speed <= .12:
                self.state = 2
                if self.straight and self.precise and self.crossing_axis_arena is not None:
                    self.exit_clear_arena=tuple(self.goal_arena[i]+.55*self.crossing_axis_arena[i] for i in range(2))
                    self.exit_axis_arena=self.crossing_axis_arena
            return (0.0, 0.0)
        sdk_speed = self.max_speed
        if axis is not None and (self.straight or
                (self.fixed_path is None and self.precise and distance<.45)):
            # The configured corridor axis is fixed. Correct lateral error
            # independently of distance to the intermediate centre goal; a
            # diagonal chord from a 5 cm entrance residual can scrape a roof.
            lateral=(-axis[1],axis[0])
            along=sum((self.goal_arena[i]-start[i])*axis[i] for i in range(2))
            forward=max(-.8,min(.8,1.2*along))
            if abs(along)>.015:forward=math.copysign(max(.2,abs(forward)),along)
            # Close the velocity loop on the exported physical speed. The new
            # road portal is a ramp: a proportional position command alone can
            # balance gravity before the centre target and stall indefinitely.
            measured_forward=robot['vx']*axis[0]+robot['vy']*axis[1]
            self.longitudinal_integral=max(-.65,min(.65,
                self.longitudinal_integral+.1*(forward-measured_forward)))
            forward=max(-.8,min(.8,forward+self.longitudinal_integral))
            measured_lateral=robot['vx']*lateral[0]+robot['vy']*lateral[1]
            if abs(cross_error)>.02:
                self.lateral_integral=max(-.2,min(.2,self.lateral_integral+.1*cross_error))
            else:self.lateral_integral=0.
            lateral_speed=max(-.5,min(.5,-4.*cross_error-2.*self.lateral_integral-1.5*measured_lateral))
            if abs(cross_error)>.02:
                # Alignment is completed before advancing under a narrow roof.
                forward=0.
                self.longitudinal_integral=0.
                if abs(measured_lateral)<.05:
                    lateral_speed=math.copysign(max(.22,abs(lateral_speed)),-cross_error)
            velocity=tuple(axis[i]*forward+lateral[i]*lateral_speed for i in range(2))
            magnitude=math.hypot(*velocity)
            if magnitude>.8:velocity=tuple(v*.8/magnitude for v in velocity)
            end=tuple(start[i]+velocity[i]*.3 for i in range(2))
            if bumps_clearance(arena,start,end,radius+.02)<=0:
                self.state=3
                self.last_failure = 'corridor_velocity_sweeps_bumps'
                return (0.,0.)
            return velocity
        if self.fixed_path is not None:
            waypoint=self.fixed_path[self.fixed_path_index]
            previous=self.fixed_path[self.fixed_path_index-1]
            dx,dy=waypoint[0]-previous[0],waypoint[1]-previous[1]
            length=math.hypot(dx,dy)
            if length<1e-9:
                dx,dy=waypoint[0]-start[0],waypoint[1]-start[1]
                length=math.hypot(dx,dy)
            tangent=(dx/length,dy/length) if length>1e-9 else (0.,0.)
            normal=(-tangent[1],tangent[0])
            remaining=sum((waypoint[i]-start[i])*tangent[i] for i in range(2))
            lateral_error=sum((start[i]-previous[i])*normal[i] for i in range(2))
            along_speed=sum(robot[k]*tangent[i] for i,k in enumerate(('vx','vy')))
            lateral_speed=sum(robot[k]*normal[i] for i,k in enumerate(('vx','vy')))
            # Track the approved segment itself, rather than repeatedly aiming
            # at its remote endpoint. Velocity feedback brakes before corners
            # and rejects the sideways ramp load that can otherwise grow into
            # a large chord across an obstacle. Near a corner keep a small
            # pass-through speed; no intermediate stop or new path is created.
            desired=math.copysign(min(self.max_speed,max(.18,abs(remaining))),remaining)
            if final_segment and self.precise and distance<.45:
                desired=math.copysign(max(.2,min(.8,1.2*abs(remaining))),remaining)
            if abs(lateral_error)>.08:
                desired*=max(0.,1.-(abs(lateral_error)-.08)/.12)
            self.longitudinal_integral=max(-.65,min(.65,
                self.longitudinal_integral+.1*(desired-along_speed)))
            forward_limit=.8 if final_segment and self.precise and distance<.45 else self.max_speed
            forward=max(-.8,min(forward_limit,
                desired+.8*(desired-along_speed)+self.longitudinal_integral))
            self.lateral_integral=max(-.3,min(.3,self.lateral_integral+.1*lateral_error))
            sideways=max(-.8,min(.8,-4.*lateral_error-1.5*lateral_speed-2.*self.lateral_integral))
            velocity=tuple(tangent[i]*forward+normal[i]*sideways for i in range(2))
            magnitude=math.hypot(*velocity)
            if magnitude>self.max_speed:
                velocity=tuple(v*self.max_speed/magnitude for v in velocity)
            end=tuple(start[i]+.3*velocity[i] for i in range(2))
            if not self._fixed_segment_clear(start,end,robot,arena,radius+.02):
                self.state=3;self.last_failure='fixed_path_blocked'
                return (0.,0.)
            return velocity
        elif self.precise and distance < .45:
            # Near an alignment point, correct the final residual directly;
            # SDK's obstacle margin must not push the target off the tunnel axis.
            dx, dy = self.goal_arena[0]-robot['x'], self.goal_arena[1]-robot['y']
            length = math.hypot(dx,dy)
            ux, uy = dx/length, dy/length
        else:
            self.plan_arena=self.plan(goal_field,robot,arena)
            if len(self.plan_arena)<2:
                self.state=3
                self.last_failure = 'no_route_via_exit' if self.exit_clear_arena is not None else 'no_ordinary_route'
                return (0.,0.)
            waypoint=self.plan_arena[1]
            dx,dy=waypoint[0]-robot['x'],waypoint[1]-robot['y']
            magnitude=math.hypot(dx,dy)
            ux,uy=(dx/magnitude,dy/magnitude) if magnitude>1e-9 else (0.,0.)
            sdk_speed=min(self.max_speed,2.*magnitude)
        # Lower speed inside the narrow corridor; stopping precision is measured.
        limit = .8 if self.straight else self.max_speed
        speed = min(limit, sdk_speed, 1.2*distance) if final_segment else min(limit,sdk_speed)
        velocity=(ux*speed,uy*speed)
        start=(robot['x'],robot['y'])
        # Look ahead beyond the 10 Hz command hold, including braking margin.
        end=(start[0]+velocity[0]*.3,start[1]+velocity[1]*.3)
        if bumps_clearance(arena,start,end,radius+.02)<=0:
            self.state=3
            self.last_failure = 'ordinary_velocity_sweeps_bumps'
            return (0.,0.)
        return velocity

class EngineFoldReport:
    """把 Arena 的**真实机构状态**翻成决策层要的折叠码。

    为什么不再计时：折叠时长由引擎按型号码的 `transform_time_s` 自己推进
    （`physics.rs`：`transform_progress = approach(..., PHYSICS_DT / transform_time_s)`），
    而且**会被几何挡住**——顶上不够高时引擎拒绝继续变形，进度停在中间。用桥自己的
    计时器回报"已折叠"就是在编数据：车其实没折下去，决策层却以为可以过洞了。
    所以这里只读引擎：`transform_progress`（0→1）+ `crouched`。

    返回值语义（与电控 B1 约定一致）：
      0 未知 / 1 折叠中 / 2 已折叠 / 3 展开中 / 4 已展开
    """

    UNKNOWN, FOLDING, FOLDED, UNFOLDING, UNFOLDED = 0, 1, 2, 3, 4

    def __init__(self):
        self._prev = None
        self._dir = None  # 上一次的变形方向；卡在中间时沿用它

    def update(self, sentry):
        """参数 sentry — 引擎这一拍的我方哨兵状态 dict。返回折叠码 0–4。副作用：记住进度。"""
        if not sentry.get("transform_allowed"):
            # 没有变形执行器的型号（如 2771）：引擎只有 `crouch` 电平，瞬间切换、没有中间态。
            if "crouched" not in sentry:
                return self.UNKNOWN
            return self.FOLDED if sentry["crouched"] else self.UNFOLDED
        progress = float(sentry.get("transform_progress", 0.0))
        prev, self._prev = self._prev, progress
        if progress >= 0.999:
            return self.FOLDED
        if progress <= 0.001:
            return self.UNFOLDED
        # 中间态：方向按进度增量判断；被挡住不动时沿用上一次方向
        if prev is not None and abs(progress - prev) > 1e-9:
            self._dir = self.FOLDING if progress > prev else self.UNFOLDING
        return self._dir or self.FOLDING

class ReplayRecorder:
    """把桥跑的这一局录成 Arena 3D 观战页能直接打开的回放文件。

    为什么需要：桥用的是**进程内嵌**的 Arena Match，和 Arena 自带的观战服务器
    （`python -m rmuc_arena serve`，默认 8765）是两条互不相通的通道——
    桥这一局不会出现在浏览器画面里。Arena 前端有个"载入回放"按钮
    （`<input type=file accept=.json>`），所以把这一局录成 `rmuc-replay-v4`
    就能拖进去看真正的 3D 画面。

    格式要求（照 `rmuc_arena/runner.py::Battle.save` 的写法和
    `frontend/src/App.tsx::loadReplay` 的校验实现）：
      - 顶层 `format` 必须是 `rmuc-replay-v4`
      - `metadata` 必须有 `teams.red` / `teams.blue`
      - 每帧要有 `robots` / `arena` / `bases` / `teams` / `events` / `projectiles`
      - 机器人 id 必须形如 `blue-7` / `red-3`；2027 赛季固定 10 台
      - 帧数 ≤ 12001；文件 ≤ 512 MiB

    体积控制：整帧约 109 KB，8400 帧（420 s @ 20 Hz）= 约 900 MB，超限。
    所以做两件事：
      1. 把 `STATIC_FRAME_KEYS`（arena / robot_models 等，占整帧 58%）提到
         static 段只存一份；
      2. 按 `hz` 降采样（默认 10 Hz，与 Arena 自己的 10 Hz 决策同频，
         前端会插值补帧，观感不受影响）。
    降采样后 420 s 约 235 MB，稳在 512 MiB 以内。
    """

    FORMAT = "rmuc-replay-v4"   # 前端只认 v2/v3/v4，v4 才允许 static 分离
    MAX_FRAMES = 12001          # 前端硬上限，超了直接判"无效的回放"

    def __init__(self, path, metadata, static, hz=10.0, step_s=0.05):
        """参数：path — 输出 .json 路径；metadata — 回放元数据；
                 static — 全场不变的键（arena / robot_models 等）；
                 hz — 采样频率 Hz，0 或负数 = 每个物理帧都存；
                 step_s — 物理步长 s（ARENA_DT，由调用方传入，别在这里假设）。"""
        self.path = Path(path)
        self.metadata = dict(metadata)
        self.static = dict(static)
        phys_hz = 1.0 / step_s if step_s > 0 else 20.0  # 物理帧频率，正常就是 20 Hz
        self.stride = max(1, int(round(phys_hz / hz))) if hz and hz > 0 else 1
        self._buf = []  # 紧凑 JSON 字节串；降采样后 4200 帧约 235 MB
        self._count = 0
        self.closed = False

    def maybe_add(self, frame):
        """按采样间隔决定要不要存这一帧。副作用：写入暂存文件。"""
        if self.closed:
            return
        if self._count % self.stride != 0:
            self._count += 1
            return
        self._count += 1
        if len(self._buf) >= self.MAX_FRAMES:
            # 帧数到顶就拉大间隔，宁可稀疏也不能让前端判无效
            self.stride *= 2
            return
        # static 里的键不重复存（v4 播放时前端会把 static 合回来）
        slim = {k: v for k, v in frame.items() if k not in self.static}
        try:
            raw = json.dumps(slim, ensure_ascii=False, separators=(",", ":"),
                             allow_nan=False).encode()
        except ValueError:
            return  # 有 NaN/inf 的帧直接丢掉，写了整份回放都会坏
        self._buf.append(raw)

    def close(self):
        """写出回放文件。副作用：落盘、关闭暂存。"""
        if self.closed:
            return
        self.closed = True
        try:
            frames = self._buf
            if not frames:
                return
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("wb") as out:
                out.write(b'{"format":"' + self.FORMAT.encode() + b'","metadata":')
                out.write(json.dumps(self.metadata, ensure_ascii=False,
                                     separators=(",", ":")).encode())
                out.write(b',"static":')
                out.write(json.dumps(self.static, ensure_ascii=False,
                                     separators=(",", ":")).encode())
                out.write(b',"frames":[')
                out.write(b",".join(frames))
                out.write(b'],"actions":[],"diagnostics":[]}')
        finally:
            self._buf = []
