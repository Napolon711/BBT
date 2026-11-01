"""Offline regression tests for simulator adapters; never opens a serial port."""
import math
import struct
import unittest
from support import FrameMap, NavController, EngineFoldReport
from serial_emulator import ArenaSerial


class AdapterTests(unittest.TestCase):
    def test_alignment_needs_position_and_settling(self):
        nav=NavController(FrameMap('blue'))
        robot={'x':18.68,'y':1.14,'vx':0.,'vy':0.}
        vx,vy=nav.update((18.68,14.),(18.68,13.86),robot,{},straight=True,precise=True)
        self.assertEqual(nav.state,1)
        self.assertGreater(math.hypot(vx,vy),0)
        robot.update(y=1.03,vx=.2)
        nav.update((18.68,14.),(18.68,13.97),robot,{},precise=True)
        self.assertEqual(nav.state,1)
        robot['vx']=0
        nav.update((18.68,14.),(18.68,13.97),robot,{},precise=True)
        self.assertEqual(nav.state,2)
        robot['y']=1.5
        nav.update((18.68,14.),(18.68,13.5),robot,{})
        self.assertEqual(nav.state,2)  # Old goal remains completed until replaced.
        nav.update((18.68,14.),(18.68,13.5),robot,{},straight=True,precise=True,revision=1)
        self.assertEqual(nav.state,1)

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
