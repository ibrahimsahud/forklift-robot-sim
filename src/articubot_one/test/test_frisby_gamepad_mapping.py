import unittest

from articubot_one.frisby_gamepad_mapping import (
    map_frisby_controls,
    pwm_to_wheel_speed,
)


class FrisbyGamepadMappingTests(unittest.TestCase):
    def test_pwm_scales_match_real_motor_conversion(self):
        self.assertAlmostEqual(pwm_to_wheel_speed(63, 150, 0.20), 0.084)
        self.assertAlmostEqual(pwm_to_wheel_speed(120, 150, 0.20), 0.16)

    def test_joystick_motion_is_fixed_pwm_63(self):
        command = map_frisby_controls([0.0, 0.25], [])
        self.assertEqual(command.selected_pwm, 63)
        self.assertEqual(command.right_selected_pwm, 63)
        self.assertAlmostEqual(command.left_wheel_m_s, 0.084)
        self.assertAlmostEqual(command.right_wheel_m_s, 0.084)

    def test_either_l1_or_r1_selects_120_only_while_held(self):
        for index in (4, 5):
            buttons = [0] * 8
            buttons[index] = 1
            command = map_frisby_controls([0.0, 1.0], buttons)
            self.assertTrue(command.turbo)
            self.assertEqual(command.selected_pwm, 120)
            self.assertEqual(command.right_selected_pwm, 120)
            self.assertAlmostEqual(command.left_wheel_m_s, 0.16)
            self.assertAlmostEqual(command.right_wheel_m_s, 0.16)
        released = map_frisby_controls([0.0, 1.0], [0] * 8)
        self.assertFalse(released.turbo)
        self.assertEqual(released.selected_pwm, 63)

    def test_hybrid_bench_mode_maps_left_63_and_right_56(self):
        command = map_frisby_controls(
            [0.0, 1.0], [], base_pwm=63, right_base_pwm=56,
        )
        self.assertEqual(command.selected_pwm, 63)
        self.assertEqual(command.right_selected_pwm, 56)
        self.assertAlmostEqual(command.left_wheel_m_s, 0.084)
        self.assertAlmostEqual(command.right_wheel_m_s, 56 * 0.20 / 150)

    def test_speed_button_alone_does_not_move_and_stick_turns(self):
        buttons = [0] * 8
        buttons[4] = 1
        stopped = map_frisby_controls([], buttons)
        self.assertEqual((stopped.left_wheel_m_s, stopped.right_wheel_m_s), (0.0, 0.0))
        right_turn = map_frisby_controls([0.5, 0.0], [0] * 8)
        self.assertAlmostEqual(right_turn.left_wheel_m_s, 0.0)
        self.assertAlmostEqual(right_turn.right_wheel_m_s, 0.084)


if __name__ == '__main__':
    unittest.main()
