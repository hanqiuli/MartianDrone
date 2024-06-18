import sys

import pytest
import numpy as np

sys.path.append('.')
import ControlAnalysis.hexacopter_flight_controller as fcnt


class TestPIDController:
    @pytest.fixture
    def pid_no_windup_no_clip(self):
        return fcnt.PIDController(Kp=1.0, Ki=0.7, Kd=0.04, wind_up_limit=None, clip_limit=np.Infinity)

    @pytest.fixture
    def pid_with_windup_no_clip(self):
        return fcnt.PIDController(Kp=1.0, Ki=0.7, Kd=0.04, wind_up_limit=200.0, clip_limit=np.Infinity)

    @pytest.fixture
    def pid_no_windup_with_clip(self):
        return fcnt.PIDController(Kp=3.0, Ki=0.5, Kd=0.09, wind_up_limit=None, clip_limit=5000.0)

    @pytest.fixture
    def pid_with_windup_with_clip(self):
        return fcnt.PIDController(Kp=3.0, Ki=0.5, Kd=0.09, wind_up_limit=800.0, clip_limit=5000.0)

    def test_no_windup_no_clip(self, pid_no_windup_no_clip):
        pid = pid_no_windup_no_clip
        measurements = np.ones(10)*50
        dt = 1.0

        expected_outputs = "-87	    -120	-155	-190	-225	-260	-295	-330	-365	-400"
        expected_outputs = expected_outputs.split()
        expected_outputs = [float(x) for x in expected_outputs]

        actual_outputs = []

        for m in measurements:
            actual_outputs.append(pid.update(m, dt))

        assert np.allclose(actual_outputs, expected_outputs, rtol=0.1), \
            f"Expected {expected_outputs}, got {actual_outputs}"

    def test_with_windup_no_clip(self, pid_with_windup_no_clip):
        pid = pid_with_windup_no_clip
        measurements = np.ones(10)*50
        dt = 1.0

        expected_outputs = "-87	    -120	-155	-190	-190	-190	-190	-190	-190	-190"
        expected_outputs = expected_outputs.split()
        expected_outputs = [float(x) for x in expected_outputs]

        actual_outputs = []

        for m in measurements:
            actual_outputs.append(pid.update(m, dt))

        assert np.allclose(actual_outputs, expected_outputs, rtol=0.1), \
            f"Expected {expected_outputs}, got {actual_outputs}"

    def test_no_windup_with_clip(self, pid_no_windup_with_clip):
        pid = pid_no_windup_with_clip
        measurements = np.ones(10)*50
        dt = 3.0

        expected_outputs = "-226.5	    -300	-375	-450	-525	-600	-675	-750	-825	-900"
        expected_outputs = expected_outputs.split()
        expected_outputs = [float(x) for x in expected_outputs]

        actual_outputs = []

        for m in measurements:
            actual_outputs.append(pid.update(m, dt))

        assert np.allclose(actual_outputs, expected_outputs, rtol=0.1), \
            f"Expected {expected_outputs}, got {actual_outputs}"

    def test_with_windup_with_clip(self, pid_with_windup_with_clip):
        pid = pid_with_windup_with_clip
        measurements = np.ones(10)*50
        dt = 4.0

        expected_outputs = "-251.125	-350	-450	-550	-550	-550	-550	-550	-550	-550"
        expected_outputs = expected_outputs.split()
        expected_outputs = [float(x) for x in expected_outputs]

        actual_outputs = []

        for m in measurements:
            actual_outputs.append(pid.update(m, dt))

        assert np.allclose(actual_outputs, expected_outputs, rtol=0.1), \
            f"Expected {expected_outputs}, got {actual_outputs}"


class TestController:
    @pytest.fixture
    def controller(self):

        # Define constants
        mass = 60.0
        
        # PID_Params given in the following structure: 
        # [P, I, D, wind_up_limit, clip_limit]
        speed_closing = 3
        precision_speed = speed_closing/10

        max_angle = 30 * np.pi/180
        
        pid_params = [
            # Angles -> Thruster_inputs
            [0.4, 0.05, 0.8, 3], # Phi
            [0.4, 0.05, 0.8, 3], # Theta
            [0.5, 0.1, 0.3, 3], # Psi
            # Positions -> Angles
            [0.18, 0.0005, 0.7, 1, precision_speed],   # X
            [0.18, 0.0005, 0.7, 1, precision_speed],   # Y
            [1.5, 0.15, 5, 3, 10],   # exception in Z: Position -> thrust
            # Velocities -> Angles
            [3, 0.008, 5, 10, max_angle],   # Velocity X
            [-3, -0.008, -5, 10, max_angle],   # Velocity Y
            [10, 0.016, 4, 20],   # exception in Z: Velocity -> thrust
            ]

        thrust_to_weight_range = [0.7, 1.3]
        skew_factor = 1.0
        # estimated_mass, pid_params
        flight_controller_args = [
            mass*skew_factor,
            pid_params
        ]

        arm_length = 2
        propellor_thrust_coefficient = 0.02
        propellor_power_coefficient = 0.0032
        propellor_radius = 1.3

        # arm_length, ENV, thrust_to_weight_range, propellor_ct, propellor_cq, propellor_r
        flight_controller_kwargs = {
            'arm_length': arm_length,
            'thrust_to_weight_range': thrust_to_weight_range,
            'propellor_thrust_coefficient': propellor_thrust_coefficient,
            'propellor_power_coefficient': propellor_power_coefficient,
            'propellor_radius': propellor_radius,
        }

        return fcnt.FlightController(*flight_controller_args, **flight_controller_kwargs)

    def test_setup_pids(self, controller):
        # Number of PIDs is 9
        assert len(controller.pid_list) == 9, 'Wrong number of PIDs'

        # maps are inverse of each other
        a = np.array(controller.pid_to_state_map)
        b = np.array(controller.state_to_pid_map)
        assert np.all(b[a] == np.arange(9)), 'PID to state map is not inverse of state to PID map'

        # Control loops are atleast 4 lists in a list, with each loop also a loop
        assert len(controller.control_loop) >= 4, 'Wrong number of control loops'
        assert all(isinstance(control_loop, list) for control_loop in controller.control_loop),  'Control loops are not lists'
        assert all(len(control_loop) == 4 for control_loop in controller.control_loop), 'Control loops are not 4 elements long'
        assert all(isinstance(loop, list) for control_loop in controller.control_loop for loop in control_loop),  'Loops in control loops are not lists'

        # Verify that the last element of each loop in all control loops corresponds to the correct state
        for control_loop in controller.control_loop:
            assert controller.pid_to_state_map[control_loop[0][-1]] in [2, 8], 'Last element of control loop does not correspond to state controlling height'
            assert controller.pid_to_state_map[control_loop[1][-1]] in [3], 'Last element of control loop does not correspond to state controlling roll'
            assert controller.pid_to_state_map[control_loop[2][-1]] in [4], 'Last element of control loop does not correspond to state controlling pitch'
            assert controller.pid_to_state_map[control_loop[3][-1]] in [5], 'Last element of control loop does not correspond to state controlling yaw'

    def test_transformation_matrix(self, controller):
        state0 = np.zeros(12)
        state0[::2] = 1

        state1 = 1-state0

        phi = np.pi/6

        assert np.allclose(controller.transform_state_rotation(state0, phi), np.array([np.sqrt(3)/2, -0.5, 1, 0, 1, 0, np.sqrt(3)/2, -0.5, 1, 0, 1, 0]))
        assert np.allclose(controller.transform_state_rotation(state1, phi), np.array([0.5, np.sqrt(3)/2,  0, 1, 0, 1, 0.5, np.sqrt(3)/2,  0, 1, 0, 1]))
    
    def test_clip_inputs(self, controller):
        # Test that the controller clips inputs to the correct range
        inputs = -np.ones(4)

        clipped_inputs = controller.clip_inputs(inputs)

        thrust = controller.thrust_to_weight_range[0]-1

        moment_max = (np.max(np.abs(controller.thrust_to_weight_range-1)))*controller.arm_length / 4
        moment = np.minimum(inputs[1:3], -moment_max)

        torque_max = 6 * np.mean(np.abs(controller.thrust_to_weight_range-1)) / controller.torque_thrust_ratio
        torque = np.minimum(inputs[3], -torque_max)

        assert np.allclose(clipped_inputs, np.array([thrust, *moment, torque]))

        inputs = np.ones(4)*0.1

        clipped_inputs = controller.clip_inputs(inputs)

        thrust = controller.thrust_to_weight_range[1]-1
        moment = inputs[1:3]
                                                                                  


