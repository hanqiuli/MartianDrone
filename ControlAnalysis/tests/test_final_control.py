import sys

import pytest
import numpy as np

sys.path.append('.')
import ControlAnalysis.Legacy_Control.final_control as fc


class TestMotorResponse:
    @pytest.fixture
    def motor(self):
        time_constant = 5.0  # example time constant
        motor = fc.MotorResponse(time_constant)
        motor.output = 0
        return motor
    
    def test_single_time_step(self, motor):
        motor.output = 10
        dt = 0.5
        input_torque = 0

        expected_output = 9
        actual_ouput = motor.get_actual_torque(input_torque, dt)

        assert np.allclose(actual_ouput, expected_output), \
            f"Expected {expected_output}, got {actual_ouput}"

    def test_step_response(self, motor):
        input_torque = 1.0  # Step input
        dt = 0.01           # Time step
        total_time = 10.0   # Total simulation time

        times = np.arange(dt, total_time+dt, dt)
        expected_outputs = input_torque * (1 - np.exp(-times / motor.time_constant))

        actual_outputs = np.zeros_like(times)
        for i in range(len(times)):
            actual_outputs[i] = motor.get_actual_torque(input_torque, dt)

        assert np.allclose(actual_outputs, expected_outputs, rtol=0.01), \
            f"Expected {expected_outputs}, got {actual_outputs}"


class TestPIDController:
    @pytest.fixture
    def pid_no_windup_no_clip(self):
        return fc.PIDController(Kp=1.0, Ki=0.7, Kd=0.04, wind_up_limit=None, clip_limit=np.Infinity)

    @pytest.fixture
    def pid_with_windup_no_clip(self):
        return fc.PIDController(Kp=1.0, Ki=0.7, Kd=0.04, wind_up_limit=200.0, clip_limit=np.Infinity)

    @pytest.fixture
    def pid_no_windup_with_clip(self):
        return fc.PIDController(Kp=3.0, Ki=0.5, Kd=0.09, wind_up_limit=None, clip_limit=5000.0)

    @pytest.fixture
    def pid_with_windup_with_clip(self):
        return fc.PIDController(Kp=3.0, Ki=0.5, Kd=0.09, wind_up_limit=800.0, clip_limit=5000.0)

    def test_no_windup_no_clip(self, pid_no_windup_no_clip):
        pid = pid_no_windup_no_clip
        measurements = np.ones(10)*50
        dt = 1.0

        expected_outputs = [1.0, 0.89, 0.77, 0.66, 0.55, 0.44, 0, 0, 0, 0]
        actual_outputs = []

        for m in measurements:
            actual_outputs.append(pid.update(m, dt))

        assert np.allclose(actual_outputs, expected_outputs, rtol=0.1), \
            f"Expected {expected_outputs}, got {actual_outputs}"

    def test_with_windup_no_clip(self, pid_with_windup_no_clip):
        pid = pid_with_windup_no_clip
        measurements = np.ones(10)*50
        dt = 1.0

        expected_outputs = [1.0, 0.89, 0.77, 0.66, 0.55, 0.44, 0, 0, 0, 0]
        actual_outputs = []

        for m in measurements:
            actual_outputs.append(pid.update(m, dt))

        assert np.allclose(actual_outputs, expected_outputs, rtol=0.1), \
            f"Expected {expected_outputs}, got {actual_outputs}"

    def test_no_windup_with_clip(self, pid_no_windup_with_clip):
        pid = pid_no_windup_with_clip
        measurements = np.ones(10)*50
        dt = 3.0

        expected_outputs = [1.0, 5.0, 5.0, 5.0, 5.0, 5.0, 0, 0, 0, 0]
        actual_outputs = []

        for m in measurements:
            actual_outputs.append(pid.update(m, dt))

        assert np.allclose(actual_outputs, expected_outputs, rtol=0.1), \
            f"Expected {expected_outputs}, got {actual_outputs}"

    def test_with_windup_with_clip(self, pid_with_windup_with_clip):
        pid = pid_with_windup_with_clip
        measurements = np.ones(10)*50
        dt = 4.0

        expected_outputs = [1.0, 5.0, 5.0, 5.0, 5.0, 5.0, 0, 0, 0, 0]
        actual_outputs = []

        for m in measurements:
            actual_outputs.append(pid.update(m, dt))

        assert np.allclose(actual_outputs, expected_outputs, rtol=0.1), \
            f"Expected {expected_outputs}, got {actual_outputs}"

