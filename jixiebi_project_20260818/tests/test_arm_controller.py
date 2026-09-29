import unittest

from arm_mole_qt.app.arm_controller import ArmController, ArmTarget


class ClosedSenderWithEvent:
    is_open = False

    def __init__(self):
        self.events = ["串口I/O连续失败：RX 已连续 3 次失败"]
        self.service_calls = 0

    def service_control(self):
        self.service_calls += 1

    def pop_transport_events(self):
        events = list(self.events)
        self.events.clear()
        return events


class VisionSenderSpy:
    is_open = True
    background_vision_enabled = True
    last_error = ""

    def __init__(self):
        self.vision_calls = []
        self.no_target_calls = 0
        self.publish_calls = []
        self.target_ctrl_calls = []

    def send_vision_error(self, dcx, dcy, valid=True):
        self.vision_calls.append((int(dcx), int(dcy), bool(valid)))
        return True

    def send_no_target(self):
        self.no_target_calls += 1
        return True

    def publish_vision_sample(self, dcx, dcy, valid, sample_time):
        self.publish_calls.append(
            (int(dcx), int(dcy), bool(valid), float(sample_time))
        )
        return True

    def send_target_ctrl(self, enabled):
        self.target_ctrl_calls.append(bool(enabled))
        return True


class ArmControllerTransportTest(unittest.TestCase):
    def test_terminal_event_is_visible_after_serial_handle_closes(self):
        controller = ArmController(enabled=False)
        sender = ClosedSenderWithEvent()
        controller.enabled = True
        controller._sender = sender

        self.assertFalse(controller.connected)
        self.assertEqual(
            controller.service_transport(),
            ["串口I/O连续失败：RX 已连续 3 次失败"],
        )
        self.assertEqual(sender.service_calls, 0)


class ArmControllerAimReferenceTest(unittest.TestCase):
    @staticmethod
    def _connected_controller(**kwargs):
        controller = ArmController(enabled=False, serial_protocol="unified", **kwargs)
        sender = VisionSenderSpy()
        controller.enabled = True
        controller._sender = sender
        return controller, sender

    def test_physical_mm_conversion_is_not_accepted_as_formal_calibration(self):
        controller = ArmController(
            enabled=False,
            laser_offset_mm=27.81,
            mm_per_pixel_x=0.125,
            laser_offset_x_px=None,
            laser_offset_y_px=9,
        )

        self.assertFalse(controller.aim_reference_configured)
        self.assertIsNone(controller.laser_offset_px())
        self.assertIsNone(controller.aim_reference_pixel((640, 480)))
        self.assertIn("未标定", controller.laser_offset_text())

    def test_aim_reference_and_error_use_explicit_pixel_offsets(self):
        controller = ArmController(
            enabled=False,
            laser_offset_x_px=70,
            laser_offset_y_px=40,
        )
        target = ArmTarget(pixel=(438, 237), frame_size=(640, 480))

        self.assertTrue(controller.aim_reference_configured)
        self.assertEqual(controller.aim_reference_pixel((640, 480)), (390, 280))
        self.assertEqual(controller.aim_corrected_error(target), (48, -43))

    def test_both_pixel_axes_must_be_explicitly_calibrated(self):
        controller = ArmController(
            enabled=False,
            laser_offset_x_px=70,
            laser_offset_y_px=None,
        )

        self.assertFalse(controller.aim_reference_configured)
        self.assertIsNone(controller.aim_reference_pixel((640, 480)))

    def test_formal_send_uses_laser_aim_reference(self):
        controller, sender = self._connected_controller(
            laser_offset_x_px=70,
            laser_offset_y_px=40,
        )
        target = ArmTarget(pixel=(400, 300), frame_size=(640, 480))

        ok, message = controller.send_target_sample(target)

        self.assertTrue(ok, message)
        self.assertEqual(sender.vision_calls, [(10, 20, True)])
        self.assertEqual(sender.no_target_calls, 0)

    def test_unconfigured_formal_send_fails_closed_even_with_mm_scale(self):
        controller, sender = self._connected_controller(
            laser_offset_mm=27.81,
            mm_per_pixel_x=0.125,
            laser_offset_x_px=None,
        )
        target = ArmTarget(pixel=(400, 300), frame_size=(640, 480))

        ok, message = controller.send_target_sample(target)

        self.assertTrue(ok, message)
        self.assertEqual(sender.vision_calls, [])
        self.assertEqual(sender.no_target_calls, 1)
        self.assertIn("valid=0", message)

    def test_background_publish_uses_reference_and_unconfigured_is_invalid(self):
        target = ArmTarget(pixel=(400, 300), frame_size=(640, 480))
        configured, configured_sender = self._connected_controller(
            laser_offset_x_px=70,
            laser_offset_y_px=40,
        )
        unconfigured, unconfigured_sender = self._connected_controller(
            laser_offset_x_px=None,
            mm_per_pixel_x=0.125,
        )

        self.assertTrue(configured.publish_target_sample(target, 12.5)[0])
        self.assertTrue(unconfigured.publish_target_sample(target, 13.5)[0])

        self.assertEqual(
            configured_sender.publish_calls,
            [(10, 20, True, 12.5)],
        )
        self.assertEqual(
            unconfigured_sender.publish_calls,
            [(0, 0, False, 13.5)],
        )

    def test_target_start_is_blocked_without_both_reference_axes(self):
        unconfigured, sender = self._connected_controller(
            laser_offset_x_px=70,
            laser_offset_y_px=None,
        )

        ok, message = unconfigured.start_target_state_machine()

        self.assertFalse(ok)
        self.assertEqual(sender.target_ctrl_calls, [])
        self.assertIn("未标定", message)
        self.assertIn("未发送", message)


if __name__ == "__main__":
    unittest.main()
