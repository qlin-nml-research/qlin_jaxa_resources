import socket
import time
import dqrobotics as dql
import numpy as np

from dqrobotics.interfaces.vrep import DQ_VrepInterface

import random

TARGET_RATE = 10
target_ip = "127.0.0.1"
# target_ip = "100.112.212.13"
target_port = 20034

arm_formatting = "x{i}: {tx} {ty} {tz} {rw} {rx} {ry} {rz} {g}"
cam_formatting = "cam: {tx} {ty} {tz} {rw} {rx} {ry} {rz}"

tool_t_random_range = np.array([0.100, 0.100, 0.400])
tool_r_random_range = np.array([0.2, 0.2, 0.2, 0.2])
cam_t_random_range = np.array([0.05, 0.05, 0.05])
cam_r_random_range = np.array([0.01, 0.01, 0.01, 0.0])

camera_center_t = dql.DQ([0, 0, 1])
camera_center_r = np.cos(np.pi / 2) + dql.i_ * np.sin(np.pi / 2)  # z pointing down
camera_center_x = camera_center_r + 0.5 * dql.E_ * camera_center_t * camera_center_r

arm_center_t = dql.DQ([0, 0, 0.4])
arm_center_r = dql.DQ([1, 0, 0, 0])
arm_center_x = arm_center_r + 0.5 * dql.E_ * arm_center_t * arm_center_r

vrep_ip = "127.0.0.1"
vrep_port = 20000

camera_name = "y_camera"
ee_names = ["xd1", "xd2"]



def generate_random_pose_with_center_pose(center_pose: dql.DQ, t_range: np.ndarray, r_range: np.ndarray):
    center_t = dql.translation(center_pose).vec3()
    center_r = dql.rotation(center_pose).vec4()

    t = center_t + np.random.uniform(-t_range, t_range)
    r = center_r + np.random.uniform(-r_range, r_range)
    r = dql.DQ(r).normalize()
    t = dql.DQ(t)
    x = r + 0.5 * dql.E_ * t * r
    return x


def main():
    socket_ = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    socket_.connect((target_ip, target_port))

    vrep = DQ_VrepInterface()
    if not vrep.connect(vrep_ip, vrep_port, 100, 10):
        print("Failed to connect to V-REP")
        return


    print(f"Connected to {target_ip}:{target_port}")
    counter = 0

    while True:
        time.sleep(1 / TARGET_RATE)

        # xs_ = [
        #     generate_random_pose_with_center_pose(arm_center_x, tool_t_random_range, tool_r_random_range),
        #     generate_random_pose_with_center_pose(arm_center_x, tool_t_random_range, tool_r_random_range)
        # ]
        # cam_x_ = generate_random_pose_with_center_pose(camera_center_x, cam_t_random_range, cam_r_random_range)

        xs_ = [vrep.get_object_pose(ee_name) for ee_name in ee_names]
        cam_x_ = vrep.get_object_pose(camera_name)

        cam_x = dql.DQ([1])
        xs = [cam_x_ * arm_x for arm_x in xs_]

        arm_strs = []
        for i, (x, arm_g) in enumerate(zip(xs, [random.uniform(0, 1), random.uniform(0, 1)])):
            arm_pose = x
            arm_t = 1000 * dql.translation(arm_pose).vec3()  # tx ty tz, In millimeters
            arm_r = dql.rotation(arm_pose).vec4()  # rw rx ry rz
            arm_str = "x{}: {} {} {} {} {} {} {} {}".format(i, arm_t[0], arm_t[1], arm_t[2], arm_r[1],
                                                            arm_r[2], arm_r[3], arm_r[0], arm_g)
            # x(id):tx ty tz rx ry rz rw g
            # print("arm_str", arm_str)
            arm_strs.append(arm_str)

        cam_t = 1000 * dql.translation(cam_x).vec3()  # tx ty tz, In millimeters
        cam_r = dql.rotation(cam_x).vec4()  # rw rx ry rz
        cam_str = "cam: {} {} {} {} {} {} {}".format(cam_t[0], cam_t[1], cam_t[2],
                                                     cam_r[1], cam_r[2], cam_r[3], cam_r[0])

        for msg in [cam_str, arm_strs[0], arm_strs[1]]:
            socket_.sendall(bytes(msg, 'utf-8'))
        print(f"Sent: {counter}")
        counter += 1


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("exit on keyboard interrupt")
