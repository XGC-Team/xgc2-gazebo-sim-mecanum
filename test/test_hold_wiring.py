#!/usr/bin/env python3
"""Integration wiring checks supplement executable Gate concurrency tests."""
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class HoldWiringTest(unittest.TestCase):
    def setUp(self):
        self.source = (ROOT / 'src/mecanum_contract_plugin.cpp').read_text()

    def test_gate_is_ready_before_command_spinner(self):
        load = self.source.split('void Load(', 1)[1].split('void Reset()', 1)[0]
        self.assertLess(load.index('hold_gate_.reset('), load.index('command_spinner_->start()'))
        self.assertLess(load.index('Hub::instance().add('), load.index('command_subscriber_ ='))

    def test_command_and_actuator_writes_are_both_serialized(self):
        command = self.source.split('void CommandCallback(', 1)[1].split('void OnUpdate(', 1)[0]
        update = self.source.split('void OnUpdate(', 1)[1].split('void ApplyPlanarVelocity(', 1)[0]
        self.assertIn('hold_gate_->withCommand', command)
        self.assertNotIn('hold_gate_->held()', command)
        self.assertIn('hold_gate_->withCommand', update)
        self.assertLess(update.index('hold_gate_->withCommand'), update.index('ApplyWheelDynamics(dt)'))
        self.assertLess(update.index('hold_gate_->withCommand'), update.index('ApplyPlanarVelocity()'))
        self.assertIn('if (held)', update)
        self.assertIn('HoldZero()', update)

    def test_shutdown_drains_both_producers_before_gate_release(self):
        shutdown = self.source.split('void Shutdown()', 1)[1].split('static void HoldZeroThunk', 1)[0]
        self.assertLess(shutdown.index('update_gate->owner = nullptr'), shutdown.index('hold_gate_.reset()'))
        self.assertLess(shutdown.index('command_spinner_->stop()'), shutdown.index('hold_gate_.reset()'))
        self.assertLess(shutdown.index('command_queue_.clear()'), shutdown.index('Hub::instance().remove'))

    def test_registry_does_not_return_unprotected_gate_pointer(self):
        udp = (ROOT / 'include/xgc_chassis_hold/udp.hpp').read_text()
        self.assertIn('registry_.apply(robot, held)', udp)
        self.assertIn('registry_.remove(gate)', udp)
        self.assertNotIn('Gate *match(', udp)
        shutdown = udp.split('~Hub()', 1)[1].split('void start()', 1)[0]
        self.assertLess(shutdown.index('thread_.join()'), shutdown.index('close(fd_)'))


if __name__ == '__main__':
    unittest.main()
