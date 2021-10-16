
import copy
import numpy as np
from tf.transformations import (
    euler_matrix,
    quaternion_from_euler,
    quaternion_from_matrix,
    quaternion_matrix,
)


"""
# RM to OLD IRM

Transform the reachability map(RM) to the inverse RM.

**Table of Contents**
1. Load data
2. RM to IRM
3. Check with plots
4. Save IRM

### 1. Load data (RM)

rzyz Euler angle (Yaw-Pitch-Roll)

| Column Index | Name              | Unit       | Remark |
| ----- | ----------------- | ---------- | ------ |
| 0     | TCP x             | meter      |        | {base} frame
| 1     | TCP y             | meter      | L!=R   | {base} frame
| 2     | TCP z             | meter      |        | {base} frame
| 3     | EEP x             | meter      |        | {base} frame
| 4     | EEP y             | meter      | L!=R   | {base} frame
| 5     | EEP z             | meter      |        | {base} frame
| 6     | EE Roll           | **DEGREE** |        | {base}2{tcp} rzyz(yaw, pitch, roll)
| 7     | EE Pitch          | **DEGREE** |        | {base}2{tcp} rzyz(yaw, pitch, roll)
| 8     | EE Yaw: Cr        | **DEGREE** | L!=R   | {base}2{tcp} rzyz(yaw, pitch, roll)
| 9     | Manipulability    | -          |        |
| 10    | Joint_0: Waist_Roll      | radian     | L!=R   |
| 11    | Joint_1: Waist_Pitch     | radian     |        |
| 12    | Joint_2: RShoulder_Pitch | radian     |        |
| 13    | Joint_3: RShoulder_Roll  | radian     | L!=R   |
| 14    | Joint_4: RElbow_Pitch    | radian     |        |
| 15    | Joint_5: RElbow_Yaw      | radian     | L!=R   |
| 16    | Joint_6: RWrist_Pitch    | radian     |        |
| 17    | Joint_7: RWrist_Roll     | radian     | L!=R   |

"robocare_right_reachability_map.npy" has 18 columns. (Index 17 is "Joint_7".)

### 2. Save data (OLD IRM)

| Column Index | Name               | Unit   | Remark       |
| ------------ | ------------------ | ------ | -----------  |
| 0            | Mobile Base x      | meter  | {tcp} frame  |
| 1            | Mobile Base y      | meter  | {tcp} frame  |
| 2            | Mobile Base z      | meter  | {tcp} frame  |
| 3            | EEP x              | meter  | {tcp} frame  |
| 4            | EEP y              | meter  | {tcp} frame  |
| 5            | EEP z              | meter  | {tcp} frame  |
| 6            | Mobile Base Quat X |        | {tcp}2{base} |
| 7            | Mobile Base Quat Y |        | {tcp}2{base} |
| 8            | Mobile Base Quat Z |        | {tcp}2{base} |
| 9            | Mobile Base Quat W |        | {tcp}2{base} |
| 10           | Manipulability     | -      |              |
| 11           | Joint_0 value      | radian |              |
| 12           | Joint_1 value      | radian |              |
| >=13 (13~18) | Joint_2... values  | radian |              |

"""

IDX_TCP_X = 0
IDX_TCP_Y = 1
IDX_TCP_Z = 2
IDX_EEF_X = 3
IDX_EEF_Y = 4
IDX_EEF_Z = 5
IDX_R = 6
IDX_P = 7
IDX_Y = 8
IDX_M = 9
IDX_Joint = 10


def rm_to_old_irm(rm):
    old_irm = []
    # tcp -> 0
    for row in rm:
        # ####### BASE Frame
        # RM
        base_tcpP = row[IDX_TCP_X:(IDX_TCP_Z + 1)]  # P
        base_eefP = row[IDX_EEF_X:(IDX_EEF_Z + 1)]
        eef_roll = np.radians(row[IDX_R])
        eef_pitch = np.radians(row[IDX_P])
        eef_yaw = np.radians(row[IDX_Y])
        # Data order: roll, pitch, yaw
        #             (rzyz: yaw-pitch-roll)

        # base -> tcp: rzyz.  y  p  r
        base2tcp_R = euler_matrix(
            eef_yaw, eef_pitch, eef_roll, axes="rzyz")  # R
        # tcp -> base: rzyz. -r -p -y
        tcp2base_R = euler_matrix(
            -eef_roll, -eef_pitch, -eef_yaw, axes="rzyz")  # R^T

        if not np.allclose(np.transpose(base2tcp_R), tcp2base_R):
            raise RuntimeError

        # ###### TCP Frame
        tcp_baseQuat = quaternion_from_matrix(tcp2base_R)
        tcp_baseR = tcp2base_R[:3, :3]             # R^T
        tcp_baseP = np.dot(-tcp_baseR, base_tcpP)  # -R^T * P
        """
        tcp_EEF = tcp_base_T * base_EEF =
        [ R^T   | -R^T*P ] [ R     | eefP ] = [ R^T*R | R^T*eefP + -R^T*P ]
        [ 0 0 0 |   1    ] [ 0 0 0 | 1    ]   [ 0 0 0 | 1 ]
        """
        tcp_eefP = np.dot(tcp_baseR, base_eefP) + tcp_baseP

        manipulability = row[IDX_M]

        old_irm.append([
            tcp_baseP[0],
            tcp_baseP[1],
            tcp_baseP[2],
            tcp_eefP[0],
            tcp_eefP[1],
            tcp_eefP[2],
            tcp_baseQuat[0],
            tcp_baseQuat[1],
            tcp_baseQuat[2],
            tcp_baseQuat[3],
            manipulability,
            row[IDX_Joint + 0],
            row[IDX_Joint + 1],
            row[IDX_Joint + 2],
            row[IDX_Joint + 3],
            row[IDX_Joint + 4],
            row[IDX_Joint + 5],
            row[IDX_Joint + 6],
            row[IDX_Joint + 7],
        ])
        # print(old_irm)
        # exit()
    return np.array(old_irm)


right_rm = np.load('robocare_right_reachability_map.npy')
# print(right_rm.shape)
# print(right_rm[:2])

rm = right_rm
old_irm = rm_to_old_irm(rm)
print(old_irm[:2])

npy_name = "robocare_right_old_irm.npy"
np.save(npy_name, old_irm)
print("JOB DONE!")
