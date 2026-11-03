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

    def test_completed_tunnel_does_not_inject_an_ordinary_exit_waypoint(self):
        nav=NavController(FrameMap('red'))
        robot={'x':4.,'y':14.,'vx':0.,'vy':0.,'team':'red'}
        corridor=('road',(2.,1.),(3.,1.),(4.,1.),(1.,0.),2.)

        class OpenArenaSDK:
            def _prepare_chapter4(self,*_):
                return [],[],lambda *_:True,lambda *_:set()

        with patch.object(support,'sdk',OpenArenaSDK()):
            nav.update((4.,1.),(4.,1.),robot,{},straight=True,precise=True,
                       revision=1,corridor=corridor)
            self.assertEqual(nav.state,2)
            self.assertEqual(nav.plan((8.,5.),robot,{}),[(4.,14.),(8.,10.)])

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

    def test_public_plan_returns_empty_when_planning_fails(self):
        nav=NavController(FrameMap('red'))
        robot={'x':2.,'y':12.,'vx':0.,'vy':0.,'team':'red','radius':.42}
        with patch.object(support,'sdk',None):
            self.assertEqual(nav.plan((5.,5.),robot,{}),[])

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
