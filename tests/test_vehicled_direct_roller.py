import threading
import unittest
from unittest.mock import patch

from robot_car.decision.action_dispatcher import ActionId
from robot_car.vehicle_link.fake_transport import FakeTransport
from robot_car.vehicled import VehicleDaemon


class FakeRollerController:
    instances = []

    def __init__(self, *_args, **kwargs):
        self.task = kwargs["task"]
        self.target_mm = kwargs["target_mm"]
        self.stop_event = threading.Event()
        self.feedforward = None
        type(self).instances.append(self)

    def run(self):
        self.stop_event.wait(1.0)

    def set_feedforward_mm_s2(self, value):
        self.feedforward = value


class VehicleDaemonDirectRollerTests(unittest.TestCase):
    def setUp(self):
        FakeRollerController.instances.clear()
        self.config = {
            "runtime": {"vision_socket": "/tmp/not-used.sock"},
            "vehicle": {"actions": {"enabled": True}, "link_timeout_ms": 500},
        }

    @patch("robot_car.vehicled.RollerControlDaemon", FakeRollerController)
    @patch("robot_car.vehicled.load_roller_control", return_value={"enabled": True})
    def test_line_balance_actions_start_direct_hold_at_dispatcher_target(self, _load):
        daemon = VehicleDaemon(self.config, FakeTransport())
        daemon.request_action(ActionId.LINE_TO_B_BALANCE_CENTER, now_ms=1000)
        first = FakeRollerController.instances[-1]
        self.assertEqual((first.task, first.target_mm), (1, 0.0))

        daemon.request_action(ActionId.LINE_LAP_BALANCE_TARGET,
                              {"target_mm": 35}, now_ms=1100)
        second = FakeRollerController.instances[-1]
        self.assertEqual((second.task, second.target_mm), (1, 35.0))
        self.assertTrue(first.stop_event.is_set())
        self.assertEqual(daemon.action_dispatcher.motion_mode().name, "LINE_FOLLOW")
        daemon._stop_roller_sweep()

    def test_direct_roller_feedforward_defaults_to_zero_when_stale_or_missing(self):
        daemon = VehicleDaemon.__new__(VehicleDaemon)
        daemon.config = {"vehicle": {"link_timeout_ms": 500}}
        controller = FakeRollerController.__new__(FakeRollerController)
        controller.task = 1
        controller.feedforward = None
        daemon._roller_sweep = controller

        daemon._update_direct_roller_feedforward({}, 1000, 1000)
        self.assertEqual(controller.feedforward, 0.0)
        daemon._update_direct_roller_feedforward({"roller_feedforward_mm_s2": 36}, 1000, 1200)
        self.assertEqual(controller.feedforward, 36)
        daemon._update_direct_roller_feedforward({"roller_feedforward_mm_s2": 36}, 1000, 1501)
        self.assertEqual(controller.feedforward, 0.0)


if __name__ == "__main__":
    unittest.main()
