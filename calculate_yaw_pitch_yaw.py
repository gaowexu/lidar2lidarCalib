import json
import numpy as np
import math


def calculate_yaw_pitch_roll_from_rotation_matrix(extrinsic_matrix_json_full_path: str):
    """
    Reference:
    https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions#Conversion_formulae_between_formalisms

    :param extrinsic_matrix_json_full_path:
    :return:
    """
    json_data = json.load(open(extrinsic_matrix_json_full_path, 'r'))

    if "left_front_lidar-to-top_center_lidar-extrinsic" in json_data.keys():
        rotation_matrix = json_data["left_front_lidar-to-top_center_lidar-extrinsic"]["param"]["sensor_calib"]["data"]
        yaw = math.atan2(rotation_matrix[1][0], rotation_matrix[0][0])
        pitch = math.atan2(-rotation_matrix[2][0], np.sqrt(rotation_matrix[2][1] ** 2 + rotation_matrix[2][2] ** 2))
        roll = math.atan2(rotation_matrix[2][1], rotation_matrix[2][2])
        print("Left -> Top-Center: Yaw = {}, Pitch = {}, Roll = {}".format(yaw, pitch, roll))
        print("ros2 run tf2_ros static_transform_publisher {} {} {} {} {} {} front left".format(
            rotation_matrix[0][3], rotation_matrix[1][3], rotation_matrix[2][3], yaw, pitch, roll))

    else:
        rotation_matrix = json_data["right_front_lidar-to-top_center_lidar-extrinsic"]["param"]["sensor_calib"]["data"]
        yaw = math.atan2(rotation_matrix[1][0], rotation_matrix[0][0])
        pitch = math.atan2(-rotation_matrix[2][0], np.sqrt(rotation_matrix[2][1] ** 2 + rotation_matrix[2][2] ** 2))
        roll = math.atan2(rotation_matrix[2][1], rotation_matrix[2][2])
        print("Right -> Top-Center: Yaw = {}, Pitch = {}, Roll = {}".format(yaw, pitch, roll))
        print("ros2 run tf2_ros static_transform_publisher {} {} {} {} {} {} front right".format(
            rotation_matrix[0][3], rotation_matrix[1][3], rotation_matrix[2][3], yaw, pitch, roll))


if __name__ == "__main__":
    calculate_yaw_pitch_roll_from_rotation_matrix(
        extrinsic_matrix_json_full_path="./extrinsic/left_lidar_to_top_center_lidar_extrinsic.json")
    calculate_yaw_pitch_roll_from_rotation_matrix(
        extrinsic_matrix_json_full_path="./extrinsic/right_lidar_to_top_center_lidar_extrinsic.json")

