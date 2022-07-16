import slampy
import cv2
import numpy as np
from utils import *
import pandas as pd
import matplotlib.pyplot as plt
import time
import os
import rosbag
import numpy as np
from IPython.display import clear_output, display
from tqdm.notebook import tqdm
from scipy.spatial.transform import Rotation as R

from numpy import sin, cos, pi

# Shift vectors from camera to robot center
shift_vecs = [
    np.array([-37.477 / 100, 7.778 / 100, 0]),
    np.array([-37.477 / 100, -7.778 / 100, 0]),
    np.array([-21. / 100, 0, 0]),
    np.array([-37.477 / 100, 7.778 / 100, 0]),
    np.array([-37.477 / 100, -7.778 / 100, 0]),
    np.array([-21. / 100, 0, 0])
]
# Rotation quaternions from camera to lidar frame
rot_quats = [
    R.from_quat([0, 0, sin(-pi/8), cos(-pi/8)]),
    R.from_quat([0, 0, sin(pi/8), cos(pi/8)]),
    R.from_quat([0, 0, sin(pi/4), cos(pi/4)]),
    R.from_quat([0, 0, sin(3*pi/8), cos(3*pi/8)]),
    R.from_quat([0, 0, sin(-3*pi/8), cos(-3*pi/8)]),
    R.from_quat([0, 0, sin(-pi/4), cos(-pi/4)])
]

# Calculate position and orientation of robot given the pos and orientation of single camera
def calc_robot_position(sv, rq, cam_num, pos, quat):
    robot_shift_vec = quat.apply(sv[cam_num])
    new_quat = quat * rq[cam_num]
    robot_pos = pos + robot_shift_vec
    return robot_pos, new_quat

# Calculate position and orientation of any camera given the pos and orientation of robot
def calc_cam_position(sv, rq, cam_num, pos, quat):
    new_quat = rq[cam_num].inv() * quat
    cam_shift_vec = new_quat.apply(sv[cam_num])
    cam_pos = pos - cam_shift_vec
    # cam_pos = pos
    return cam_pos, new_quat

def load_image_from_msg(msg):
    im = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
    return im

def calc_trajectory(app:slampy.System, bag:rosbag.Bag, topics, cam_num=None):
    positions = []
    quaternions = []
    raw_outputs = []
    timestamps = []
    
    current_frame_num = 0
    conseq_ok = 0
    conseq_not_ok = 1
    overall_ok = 0
    overall_not_ok = 0
    
    for topic, msg, t in bag.read_messages(topics=topics):

        current_frame_num += 1

        im = load_image_from_msg(msg)
        state = app.process_image_mono(im,t.to_sec() + timestamp_shift)

        if state == slampy.State.OK:
            conseq_not_ok = 0
            conseq_ok += 1
            overall_ok += 1

            new_pose = app.get_pose_to_target()
            # FIXED THIS GODDAMN XZY
            # raw_p = new_pose[0:3, 3].flatten()[[0,2,1]]
            raw_p = new_pose[0:3, 3].flatten()[[2,0,1]]
            raw_r = R.from_quat(R.from_matrix(new_pose[:3, :3]).as_quat()[[2,0,1,3]])
            if cam_num == None:
                new_p = raw_p
                new_r = raw_r
            else:
                new_p, new_r = calc_robot_position(shift_vecs, rot_quats, cam_num, raw_p, raw_r)
            positions.append(new_p)
            quaternions.append(new_r.as_quat())
            raw_outputs.append(new_pose)
            timestamps.append(t.to_nsec())
        else:
            conseq_not_ok += 1
            conseq_ok = 0
            overall_not_ok += 1

    return positions, quaternions, raw_outputs, timestamps
