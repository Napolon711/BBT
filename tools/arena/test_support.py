"""Offline regression tests for simulator adapters; never opens a serial port."""
import math
import struct
import unittest
from unittest.mock import patch

try:
    from . import support
    from .support import FrameMap, NavController, EngineFoldReport, TunnelMonitor
    from .serial_emulator import ArenaSerial
    from .navigation_constraints import bumps_clearance, sentry_routing_map
    from .nav_plan import remaining_route
except ImportError:
    import support
    from support import FrameMap, NavController, EngineFoldReport, TunnelMonitor
    from serial_emulator import ArenaSerial
    from navigation_constraints import bumps_clearance, sentry_routing_map
    from nav_plan import remaining_route


class AdapterTests(unittest.TestCase):
    def test_alignment_needs_position_and_settling(self):
        nav=NavController(FrameMap('blue'))
        corridor=('road',(18.68,14.),(18.68,12.),(18.68,10.),(0.,-1.),4.)
        robot={'x':18.68,'y':1.14,'vx':0.,'vy':0.,'team':'blue'}
        vx,vy=nav.update((18.68,14.),(18.68,13.86),robot,{},straight=True,
                         precise=True,corridor=corridor)
        self.assertEqual(nav.state,1)
        self.assertGreater(math.hypot(vx,vy),0)
        # A stationary 3 cm residual still exceeds the current 2.5 cm target.
        robot.update(y=1.03)
        nav.update((18.68,14.),(18.68,13.97),robot,{},precise=True)
        self.assertEqual(nav.state,1)
        # Being inside that target is insufficient while physical motion persists.
        robot.update(y=1.02,vx=.2)
        self.assertEqual(nav.update((18.68,14.),(18.68,13.98),robot,{},precise=True),(0.,0.))
        self.assertEqual(nav.state,1)
        robot['vx']=0
        nav.update((18.68,14.),(18.68,13.98),robot,{},precise=True)
        self.assertEqual(nav.state,2)
        robot['y']=1.5
        nav.update((18.68,14.),(18.68,13.5),robot,{})
        self.assertEqual(nav.state,2)  # Old goal remains completed until replaced.
        nav.update((18.68,14.),(18.68,13.5),robot,{},straight=True,precise=True,
                   revision=1,corridor=corridor)
        self.assertEqual(nav.state,1)

    def test_center_and_exit_are_authorized_corridor_goals(self):
        monitor=TunnelMonitor([{'name':'road','center':[2.,3.],
            'align':[{'p':[2.,1.]},{'p':[2.,5.]}],
            'corners':[[1.6,1.5],[2.4,1.5],[2.4,4.5],[1.6,4.5]]}])
        self.assertTrue(monitor.is_alignment((2.,3.)))
        self.assertEqual(monitor.crossing(2.,1.,2.,3.),'road')
        self.assertEqual(monitor.crossing(2.,3.,2.,5.),'road')
        self.assertEqual(monitor.crossing(2.,3.,2.,1.),'road')
        self.assertIsNone(monitor.crossing(2.,3.,3.,3.))
        self.assertIsNone(monitor.crossing(2.5,3.,2.,5.))

    def test_corridor_aligns_laterally_before_advancing(self):
        monitor=TunnelMonitor([{'name':'road','center':[2.,3.],
            'align':[{'p':[2.,1.]},{'p':[2.,5.]}]}])
        nav=NavController(FrameMap('blue'))
        robot={'x':2.05,'y':14.,'vx':0.,'vy':0.,'team':'blue'}
        vx,vy=nav.update((2.,3.),(2.05,1.),robot,{},straight=True,precise=True,
                         corridor=monitor.corridors[0])
        self.assertLess(vx,0.)
        self.assertAlmostEqual(vy,0.)
        self.assertEqual(nav.state,1)

    def test_bumps_remain_physical_but_are_forbidden_to_navigation(self):
        arena={'layout_revision':'test','terrain':[{'id':'bump','kind':'bumps',
            'x':3.,'y':3.,'w':1.,'h':1.}],'obstacles':[]}
        routing=sentry_routing_map(arena)
        self.assertEqual(arena['obstacles'],[])
        self.assertEqual(routing['terrain'],arena['terrain'])
        self.assertEqual(len(routing['obstacles']),1)
        self.assertNotEqual(routing['layout_revision'],arena['layout_revision'])
        self.assertLessEqual(bumps_clearance(arena,(2.,3.5),(5.,3.5),.47),0.)
        self.assertGreater(bumps_clearance(arena,(2.,2.),(5.,2.),.47),0.)
        nav=NavController(FrameMap('blue'))
        robot={'x':2.,'y':3.5,'vx':0.,'vy':0.,'team':'blue'}
        self.assertEqual(nav.update((3.5,11.5),(2.,11.5),robot,arena), (0.,0.))
        self.assertEqual(nav.state,3)

    def test_missing_route_graph_fails_closed(self):
        robot={'x':1.,'y':1.,'team':'blue'}
        self.assertEqual(remaining_route(None,robot,(2.,2.),{}),[])
        nav=NavController(FrameMap('blue'))
        robot.update(vx=0.,vy=0.,transform_progress=1.)
        with patch.object(support,'sdk',None):
            self.assertEqual(nav.update((2.,13.),(1.,14.),robot,{}),(0.,0.))
        self.assertFalse(nav.straight)  # Folding never authorizes a straight path.
        self.assertEqual(nav.state,3)
        self.assertEqual(nav.last_failure,'no_ordinary_route')

    def test_exit_connector_remains_until_ordinary_route_is_clear(self):
        nav=NavController(FrameMap('red'))
        connector=(16.3,14.44)
        goal=(21.3,12.49)
        nav.exit_clear_arena=connector
        nav.exit_axis_arena=(1.,0.)
        robot={'x':16.15937766,'y':14.47224892,'vx':.896,'vy':0.,'team':'red'}
        def route(sdk,rb,destination,arena,radius):
            start=(rb['x'],rb['y'])
            # The wall-end inflated envelope blocks the diagonal from the
            # first measured pose, though the existing axial connector clears it.
            if destination==goal and start[0]<16.18:
                return []
            return [start,destination]
        with patch.object(support,'remaining_route',side_effect=route):
            velocity=nav.update((21.3,2.51),(robot['x'],15-robot['y']),robot,{})
            self.assertEqual(nav.state,1)
            self.assertEqual(nav.exit_clear_arena,connector)
            self.assertEqual(nav.plan_arena[1],connector)
            self.assertGreater(velocity[0],0.)
            robot['x']=16.20149610
            nav.update((21.3,2.51),(robot['x'],15-robot['y']),robot,{})
            self.assertEqual(nav.state,1)
            self.assertIsNone(nav.exit_clear_arena)
            self.assertEqual(nav.plan_arena[-1],goal)

    def test_plan_returns_complete_arena_path_without_changing_waypoints(self):
        nav=NavController(FrameMap('red'))
        robot={'x':2.,'y':12.,'vx':0.,'vy':0.,'team':'red','radius':.42}
        arena={}
        # The planner is deliberately exercised at the public seam: its
        # complete Arena route retains all of the graph's intermediate vertices.
        route=[(2.,12.),(3.,11.),(5.,10.)]
        with patch.object(support,'sdk',object()), patch.object(
                support,'remaining_route',return_value=route):
            self.assertEqual(nav.plan((5.,5.),robot,arena),route)

    def test_fixed_path_does_not_replan_or_stop_at_intermediate_waypoints(self):
        nav=NavController(FrameMap('red'))
        robot={'x':2.,'y':12.,'vx':0.,'vy':0.,'team':'red','radius':.42}
        path=[(2.,3.),(3.,4.),(5.,5.)]
        arena={'terrain':[]}
        with patch.object(support,'remaining_route',side_effect=AssertionError('must not replan')):
            first=nav.update((5.,5.),(2.,3.),robot,arena,revision=1,
                             fixed_path=path)
        self.assertGreater(first[0],0.)
        self.assertEqual(nav.state,1)
        # At the first waypoint, follow the next leg immediately rather than
        # reporting success or requiring a stop at that intermediate point.
        robot.update(x=3.,y=11.)
        second=nav.update((5.,5.),(3.,4.),robot,arena,revision=1,
                          fixed_path=path)
        self.assertGreater(second[0],0.)
        self.assertEqual(nav.state,1)
        self.assertEqual(nav.plan_arena,[(2.,12.),(3.,11.),(5.,10.)])
        # Entering the final tolerance with physical velocity is still Running.
        robot.update(x=5.,y=10.,vx=.3)
        self.assertEqual(nav.update((5.,5.),(5.,5.),robot,arena,revision=1,
                                    fixed_path=path),(0.,0.))
        self.assertEqual(nav.state,1)
        robot['vx']=0.
        nav.update((5.,5.),(5.,5.),robot,arena,revision=1,fixed_path=path)
        self.assertEqual(nav.state,2)

    def test_fixed_path_fails_when_start_is_far_or_path_hits_bumps(self):
        nav=NavController(FrameMap('red'))
        robot={'x':2.,'y':12.,'vx':0.,'vy':0.,'team':'red','radius':.42}
        path=[(4.,3.),(5.,5.)]
        self.assertEqual(nav.update((5.,5.),(2.,3.),robot,{'terrain':[]},
                                    revision=1,fixed_path=path),(0.,0.))
        self.assertEqual(nav.state,3)
        self.assertEqual(nav.last_failure,'fixed_path_start_deviation')
        nav=NavController(FrameMap('red'))
        robot['x']=2.;robot['y']=12.
        blocked={'terrain':[{'id':'b','kind':'bumps','x':2.5,'y':11.5,
                             'w':1.,'h':1.}]}
        blocked_path=[(2.,3.),(5.,3.)]
        self.assertEqual(nav.update((5.,3.),(2.,3.),robot,blocked,
                                    revision=1,fixed_path=blocked_path),(0.,0.))
        self.assertEqual(nav.state,3)
        self.assertEqual(nav.last_failure,'fixed_path_blocked')

    def test_fixed_path_cannot_shortcut_after_deviating_from_approved_segment(self):
        nav=NavController(FrameMap('red'))
        robot={'x':2.,'y':12.,'vx':0.,'vy':0.,'team':'red','radius':.42}
        path=[(2.,3.),(3.,4.),(5.,5.)]
        nav.update((5.,5.),(2.,3.),robot,{},revision=1,fixed_path=path)
        robot.update(x=4.,y=12.)
        self.assertEqual(nav.update((5.,5.),(4.,3.),robot,{},revision=1,
                                    fixed_path=path),(0.,0.))
        self.assertEqual(nav.state,3)
        self.assertEqual(nav.last_failure,'fixed_path_tracking_deviation')

    def test_fixed_path_nearby_endpoint_does_not_skip_required_intermediate_route(self):
        nav=NavController(FrameMap('red'))
        robot={'x':2.,'y':12.,'vx':0.,'vy':0.,'team':'red','radius':.42}
        path=[(2.,3.),(3.,3.),(3.,4.),(2.02,3.)]
        velocity=nav.update((2.02,3.),(2.,3.),robot,{},revision=1,fixed_path=path)
        self.assertEqual(nav.state,1)
        self.assertGreater(velocity[0],0.)
        robot.update(x=3.)
        nav.update((2.02,3.),(3.,3.),robot,{},revision=1,fixed_path=path)
        self.assertEqual(nav.state,1)
        robot.update(y=11.)
        nav.update((2.02,3.),(3.,4.),robot,{},revision=1,fixed_path=path)
        self.assertEqual(nav.state,1)
        robot.update(x=2.02,y=12.)
        self.assertEqual(nav.update((2.02,3.),(2.02,3.),robot,{},revision=1,
                                    fixed_path=path),(0.,0.))
        self.assertEqual(nav.state,2)

    def test_public_plan_returns_empty_when_planning_fails(self):
        nav=NavController(FrameMap('red'))
        robot={'x':2.,'y':12.,'vx':0.,'vy':0.,'team':'red','radius':.42}
        with patch.object(support,'sdk',None):
            self.assertEqual(nav.plan((5.,5.),robot,{}),[])

    def test_precise_fixed_path_reaches_target_under_constant_uphill_load(self):
        nav=NavController(FrameMap('red'))
        robot={'x':2.,'y':11.0535,'vx':0.,'vy':0.,'team':'red','radius':.42}
        path=[(2.,3.9465),(2.,4.)]
        # A small constant uphill load balances the old proportional controller
        # before the 2.5 cm terminal tolerance. Exercise the public controller
        # against measured movement, rather than inspecting a PID accumulator.
        for _ in range(80):
            pose=(robot['x'],15.-robot['y'])
            vx,vy=nav.update((2.,4.),pose,robot,{},precise=True,
                             revision=1,fixed_path=path)
            if nav.state==2:
                break
            actual=max(0.,-vy-.08)
            robot.update(vx=vx,vy=-actual,y=robot['y']-.1*actual)
        self.assertEqual(nav.state,2)
        self.assertLessEqual(math.dist((robot['x'],robot['y']),(2.,11.)),.025)
        self.assertLessEqual(math.hypot(robot['vx'],robot['vy']),.12)
        self.assertEqual(nav.plan_arena,[(2.,11.0535),(2.,11.)])

    def test_fixed_path_handles_inertia_and_cross_slope_without_cutting_corners(self):
        nav=NavController(FrameMap('red'))
        robot={'x':2.,'y':12.,'vx':0.,'vy':0.,'team':'red','radius':.42}
        path=[(2.,3.),(14.,3.),(14.,11.)]
        arena_path=[(2.,12.),(14.,12.),(14.,4.)]
        closest_to_corner=math.inf
        for tick in range(1800):
            if tick%2==0:
                velocity=nav.update(path[-1],(robot['x'],15.-robot['y']),robot,{},
                    precise=True,revision=1,fixed_path=path)
            if nav.state in (2,3):
                break
            # The chassis cannot instantaneously turn or brake; a small cross
            # slope additionally pushes it away from a long approved segment.
            drift=.2 if robot['x']<13.5 else 0.
            for key,command,load in (('vx',velocity[0],0.),('vy',velocity[1],drift)):
                robot[key]+=.05/.3*(.75*command+load-robot[key])
            robot['x']+=.05*robot['vx'];robot['y']+=.05*robot['vy']
            closest_to_corner=min(closest_to_corner,math.dist((robot['x'],robot['y']),arena_path[1]))
            deviation=min(support.point_segment_distance((robot['x'],robot['y']),a,b)
                          for a,b in zip(arena_path,arena_path[1:]))
            self.assertLessEqual(deviation,.3)
        self.assertEqual(nav.state,2)
        self.assertLessEqual(closest_to_corner,.08)
        self.assertEqual(nav.plan_arena,arena_path)

    def test_fixed_path_observed_motion_passes_corner_between_control_updates(self):
        nav=NavController(FrameMap('red'))
        robot={'x':2.,'y':12.,'vx':0.,'vy':0.,'team':'red','radius':.42}
        path=[(2.,3.),(2.5,3.),(2.5,5.)]
        nav.update(path[-1],(2.,3.),robot,{},revision=1,fixed_path=path)
        robot.update(x=2.4,y=11.93,vx=.8)
        nav.update(path[-1],(2.4,3.07),robot,{},revision=1,fixed_path=path)
        # Neither sample enters the 8 cm disk, but their observed physical
        # swept segment passes within 7 cm. Follow the next approved segment.
        robot.update(x=2.6,y=11.93)
        velocity=nav.update(path[-1],(2.6,3.07),robot,{},revision=1,fixed_path=path)
        self.assertEqual(nav.state,1)
        self.assertLess(velocity[1],0.)
        self.assertEqual(nav.plan_arena,[(2.,12.),(2.5,12.),(2.5,10.)])

    def test_fixed_path_recovers_to_segment_when_remote_endpoint_chord_is_blocked(self):
        nav=NavController(FrameMap('red'))
        robot={'x':2.,'y':12.,'vx':0.,'vy':0.,'team':'red','radius':.42}
        path=[(2.,3.),(12.,3.)]
        def clear(a,b,ignore,strict):
            # The approved line and local recovery are free. A chord from the
            # laterally displaced measured pose to the remote endpoint is not.
            return not (a[0]>2. and abs(a[1]-12.)>.01 and b==(12.,12.))
        class GraphSDK:
            def _prepare_chapter4(self,*args):
                return [],[],clear,lambda *args:set()
        with patch.object(support,'sdk',GraphSDK()):
            nav.update(path[-1],(2.,3.),robot,{},revision=1,fixed_path=path)
            robot.update(x=4.,y=12.05,vx=1.5)
            velocity=nav.update(path[-1],(4.,2.95),robot,{},revision=1,fixed_path=path)
        self.assertEqual(nav.state,1)
        self.assertLess(velocity[1],0.)
        self.assertEqual(nav.plan_arena,[(2.,12.),(12.,12.)])

    def test_fixed_path_rejects_collision_during_local_recovery(self):
        nav=NavController(FrameMap('red'))
        robot={'x':2.,'y':12.,'vx':0.,'vy':0.,'team':'red','radius':.42}
        path=[(2.,3.),(12.,3.)]
        class GraphSDK:
            def _prepare_chapter4(self,*args):
                return [],[],lambda a,b,*args:abs(a[1]-12.)<.01,lambda *args:set()
        with patch.object(support,'sdk',GraphSDK()):
            nav.update(path[-1],(2.,3.),robot,{},revision=1,fixed_path=path)
            robot.update(x=4.,y=12.05,vx=1.5)
            velocity=nav.update(path[-1],(4.,2.95),robot,{},revision=1,fixed_path=path)
        self.assertEqual(velocity,(0.,0.))
        self.assertEqual(nav.state,3)
        self.assertEqual(nav.last_failure,'fixed_path_blocked')

    def test_fold_uses_physics(self):
        fold=EngineFoldReport()
        self.assertEqual(fold.update({'transform_allowed':True,'transform_progress':0.}),4)
        self.assertEqual(fold.update({'transform_allowed':True,'transform_progress':.5}),1)
        self.assertEqual(fold.update({'transform_allowed':True,'transform_progress':1.}),2)
        self.assertEqual(fold.update({'transform_allowed':True,'transform_progress':.5}),3)
        self.assertEqual(fold.update({'transform_allowed':True,'transform_progress':0.}),4)

    def test_remote_counter_and_paid_revive(self):
        # Bypass I/O construction: this test calls only the pure byte consumer.
        serial=ArenaSerial.__new__(ArenaSerial)
        serial.robot_id=107;serial.counts={'ammo':0,'hp':0};serial.order=None
        serial.inflight=None;serial.bad_commands=0;serial.revive_requested=False
        def wire(bits):
            return struct.pack('<HHB3xHHHI',0x301,10,0,0x120,107,0x8080,bits).ljust(136,b'\0')
        serial.receive_purchase(wire(1<<13))
        robot={'alive':True,'pending_ammo':[],'pending_heal':None}
        self.assertEqual(serial.purchase_action(robot),{'buy_ammo':{'amount':100,'remote':True}})
        robot['pending_ammo']=[(6,100)];serial.observe_purchase(robot)
        self.assertEqual(serial.counts['ammo'],1)
        serial.receive_purchase(wire(1<<13))
        self.assertEqual(serial.purchase_action(robot),{})
        serial.receive_purchase(wire((1<<13)|3))
        robot['alive']=False
        self.assertEqual(serial.purchase_action(robot),{'revive':True})
        robot['alive']=True
        self.assertEqual(serial.purchase_action(robot),{})
        self.assertEqual(serial.bad_commands,0)

if __name__=='__main__':
    unittest.main()
