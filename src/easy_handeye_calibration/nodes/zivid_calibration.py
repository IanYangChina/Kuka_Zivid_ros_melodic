import datetime
from typing import List

import numpy as np
import zivid
import os
import open3d as o3d


def construct_homogeneous_transform_matrix(translation, orientation):
    translation = np.array(translation).reshape((3, 1))  # xyz
    if len(orientation) == 4:
        rotation = o3d.geometry.get_rotation_matrix_from_quaternion(np.array(orientation).reshape((4, 1)))  # wxyz
    else:
        assert len(orientation) == 3, 'orientation should be a quaternion or 3 axis angles'
        rotation = np.deg2rad(np.array(orientation).astype("float")).reshape((3, 1))  # CBA in radians
        rotation = o3d.geometry.get_rotation_matrix_from_zyx(rotation)
    transformation = np.append(rotation, translation, axis=1)
    transformation = np.append(transformation, np.array([[0, 0, 0, 1]]), axis=0)
    return transformation


def _enter_robot_pose(index: int) -> zivid.calibration.Pose:
    """Robot pose user input.

    Args:
        index: Robot pose ID

    Returns:
        robot_pose: Robot pose

    """
    x = float(input("X (in mm): "))
    y = float(input("Y (in mm): "))
    z = float(input("Z (in mm): "))
    a = float(input("A (rotate about X, in degree): "))
    b = float(input("B (rotate about Y, in degree): "))
    c = float(input("C (rotate about Z, in degree): "))
    translation = np.array([x, y, z])
    orientation = np.array([c, b, a])
    transform_base_to_ee = construct_homogeneous_transform_matrix(translation=translation,
                                                                  orientation=orientation)
    robot_pose = zivid.calibration.Pose(transform_base_to_ee)
    print(f"The following pose was entered:\n{robot_pose}")
    return robot_pose


def _perform_calibration(hand_eye_input: List[zivid.calibration.HandEyeInput]) -> zivid.calibration.HandEyeOutput:
    """Hand-Eye calibration type user input.

    Args:
        hand_eye_input: Hand-Eye calibration input

    Returns:
        hand_eye_output: Hand-Eye calibration result

    """
    while True:
        calibration_type = input("Enter type of calibration, eth (for eye-to-hand) or eih (for eye-in-hand):").strip()
        if calibration_type.lower() == "eth":
            print("Performing eye-to-hand calibration")
            hand_eye_output = zivid.calibration.calibrate_eye_to_hand(hand_eye_input)
            return hand_eye_output
        if calibration_type.lower() == "eih":
            print("Performing eye-in-hand calibration")
            hand_eye_output = zivid.calibration.calibrate_eye_in_hand(hand_eye_input)
            return hand_eye_output
        print(f"Unknown calibration type: '{calibration_type}'")


def _assisted_capture(camera: zivid.Camera) -> zivid.Frame:
    """Acquire frame with capture assistant.

    Args:
        camera: Zivid camera

    Returns:
        frame: Zivid frame

    """
    suggest_settings_parameters = zivid.capture_assistant.SuggestSettingsParameters(
        max_capture_time=datetime.timedelta(milliseconds=800),
        ambient_light_frequency=zivid.capture_assistant.SuggestSettingsParameters.AmbientLightFrequency.none,
    )
    settings = zivid.capture_assistant.suggest_settings(camera, suggest_settings_parameters)
    return camera.capture(settings)


def _main() -> None:
    app = zivid.Application()

    print("Connecting to camera")
    camera = app.connect_camera()

    current_pose_id = 0
    hand_eye_input = []
    calibrate = False

    while not calibrate:
        command = input("Enter command, p (to add robot pose) or c (to perform calibration):").strip()
        if command == "p":
            try:
                robot_pose = _enter_robot_pose(current_pose_id)

                frame = _assisted_capture(camera)

                print("Detecting checkerboard in point cloud")
                detection_result = zivid.calibration.detect_feature_points(frame.point_cloud())

                if detection_result.valid():
                    print("Calibration board detected")
                    hand_eye_input.append(zivid.calibration.HandEyeInput(robot_pose, detection_result))
                    current_pose_id += 1
                else:
                    print(
                        "Failed to detect calibration board, ensure that the entire board is in the view of the camera"
                    )
            except ValueError as ex:
                print(ex)
        elif command == "c":
            calibrate = True
        else:
            print(f"Unknown command '{command}'")

    calibration_result = _perform_calibration(hand_eye_input)
    transform = calibration_result.transform()
    transform_file_path = os.path.join(os.path.dirname(__file__), '..', 'results_eye_on_hand', 'Extrinsic_zivid.npy')
    np.save(os.path.join(transform_file_path), transform)

    if calibration_result.valid():
        print("Hand-Eye calibration OK")
        print(f"Result:\n{calibration_result}")
    else:
        print("Hand-Eye calibration FAILED")


if __name__ == "__main__":
    _main()
