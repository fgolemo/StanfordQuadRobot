import pickle

import numpy as np
import time
import os

from pupper.HardwareInterfaceArm import HardwareInterfaceArm
from src.IMU import IMU
from src.Controller import Controller
from src.JoystickInterface import JoystickInterface
from src.State import State, ArmState, GripperState
from pupper.HardwareInterface import HardwareInterface
from pupper.Config import Configuration
from pupper.Kinematics import four_legs_inverse_kinematics


def main(use_imu=False):
    """Main program
    """

    # Create config
    config = Configuration()
    hardware_interface = HardwareInterface()
    hardware_interface_arm = HardwareInterfaceArm()
    hardware_interface_arm.start()

    # Create imu handle
    if use_imu:
        imu = IMU(port="/dev/ttyACM0")
        imu.flush_buffer()

    # Create controller and user input handles
    controller = Controller(config, four_legs_inverse_kinematics, )
    state = State()
    print("Creating joystick listener...")
    joystick_interface = JoystickInterface(config)
    print("Done.")

    last_loop = time.time()

    print("Summary of gait parameters:")
    print("overlap time: ", config.overlap_time)
    print("swing time: ", config.swing_time)
    print("z clearance: ", config.z_clearance)
    print("x shift: ", config.x_shift)

    currently_recording = False
    currently_running_arm = False
    recording_pause_timer = time.time()
    states = {}
    state.arm_joint_angles[:] = hardware_interface_arm.pos_rest
    hardware_interface_arm.move(state.arm_joint_angles)

    # Wait until the activate button has been pressed
    while True:
        print("Waiting for L1 to activate robot.")
        while True:
            command = joystick_interface.get_command(state)
            joystick_interface.set_color(config.ps4_deactivated_color)
            if command.activate_event == 1:
                break
            time.sleep(0.1)
        print("Robot activated.")
        joystick_interface.set_color(config.ps4_color)


        while True:
            now = time.time()
            if now - last_loop < config.dt:
                continue
            last_loop = time.time()

            # Parse the udp joystick commands and then update the robot controller's parameters
            command = joystick_interface.get_command(state)
            if command.activate_event == 1:
                print("Deactivating Robot")
                break

            if state.arm_state is not ArmState.DEACTIVATED:
                if not currently_running_arm:
                    joystick_interface.set_color(config.ps4_arm_color)
                    if (state.arm_joint_angles == hardware_interface_arm.pos_rest).all():
                        state.arm_joint_angles = hardware_interface_arm.pos_erect
                currently_running_arm = True
                hardware_interface_arm.move(state.arm_joint_angles)
            else:
                if currently_running_arm:
                    joystick_interface.set_color(config.ps4_color)
                    currently_running_arm = False


            if command.record_event == 1 and not currently_recording and (time.time() - recording_pause_timer) > 1:
                print("starting recording")
                joystick_interface.set_color(config.ps4_record_color)
                currently_recording = True
                recording_pause_timer = time.time()
                states = {}
            elif command.record_event == 1 and currently_recording and (time.time() - recording_pause_timer) > 1:
                print("finishing recording")
                joystick_interface.set_color(config.ps4_color)
                currently_recording = False
                with open(f"/home/pi/recordings/{time.time()}.pkl", "wb") as handle:
                    pickle.dump(states, handle, protocol=pickle.HIGHEST_PROTOCOL)
                del states
                recording_pause_timer = time.time()

            if command.gripper_event == 1:

                if state.gripper_state is GripperState.NEUTRAL:
                    hardware_interface_arm.gripper_open()
                elif state.gripper_state is GripperState.OPEN:
                    hardware_interface_arm.gripper_close()
                elif state.gripper_state is GripperState.CLOSED:
                    hardware_interface_arm.gripper_neutral()

            # Read imu data. Orientation will be None if no data was available
            quat_orientation = imu.read_orientation() if use_imu else np.array([1, 0, 0, 0])
            state.quat_orientation = quat_orientation

            # Step the controller forward by dt
            controller.run(state, command)

            if currently_recording:
                states[str(time.time())] = {"feet": np.copy(state.foot_locations),
                                            "joints": np.copy(state.joint_angles), "height": np.copy(state.height),
                                            "yaw": np.copy(state.yaw_rate), "pitch": np.copy(state.pitch),
                                            "roll": np.copy(state.roll)}

            # Update the pwm widths going to the servos
            hardware_interface.set_actuator_postions(state.joint_angles)


main()
