import threading
import unittest
from unittest.mock import patch

from robot_car.decision.action_dispatcher import ActionId
from robot_car.perception.events import VisionEvent
from robot_car.protocol.framing import MAX_PAYLOAD
from robot_car.protocol.messages import pack_json
from robot_car.vehicle_link.fake_transport import FakeTransport
from robot_car.vehicled import VehicleDaemon, build_action_status_payload


class FakeRollerController:
    instances = []

    def __init__(self, *_args, **kwargs):
        self.task = kwargs["task"]
        self.target_mm = kwargs["target_mm"]
        self.external_vision_events = kwargs["external_vision_events"]
        self.stop_event = threading.Event()
        self.feedforward = None
        self.events = []
        type(self).instances.append(self)

    def run(self):
        self.stop_event.wait(1.0)

    def set_feedforward_mm_s2(self, value):
        self.feedforward = value

    def submit_vision_event(self, event):
        self.events.append(event)


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
        self.assertTrue(first.external_vision_events)
        event = VisionEvent(1001, "roller_balance", "BALL_BALANCE_STATE", 0.95,
                            {"error_mm": 12}, 1, 120, 1280, 720)
        daemon._forward_direct_roller_event(event)
        self.assertEqual(first.events, [event])

        daemon.request_action(ActionId.LINE_LAP_BALANCE_TARGET,
                              {"operation": "set"}, now_ms=1100)
        self.assertTrue(first.stop_event.is_set())
        self.assertEqual(len(FakeRollerController.instances), 1)
        daemon.action_dispatcher.handle_event(
            VisionEvent(1101, "roller_balance", "BALL_BALANCE_STATE", 0.95,
                        {"error_mm": 35}, 1, 120, 1280, 720), now_ms=1101)
        daemon.request_action(ActionId.LINE_LAP_BALANCE_TARGET,
                              {"operation": "run", "target_revision": 1}, now_ms=1200)
        second = FakeRollerController.instances[-1]
        self.assertEqual((second.task, second.target_mm), (1, 35.0))
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

    def test_action_status_payload_stays_within_the_protocol_limit(self):
        payload = build_action_status_payload({
            "action_id": None,
            "last_action_id": 6,
            "status": "COMPLETE",
            "target_mm": -1000.0,
            "target_revision": 65535,
            "request_id": 65535,
            "reason": "\u6d4b\u8bd5" * 100,
        })
        event = {"event_type": "ACTION_STATUS", "payload": payload, "valid_for_ms": 1000}
        self.assertEqual(payload["action_id"], 6)
        self.assertLessEqual(len(pack_json(event)), MAX_PAYLOAD)


if __name__ == "__main__":
    unittest.main()
