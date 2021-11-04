# RM

rzyz Euler angle (Yaw-Pitch-Roll)

| Column Index | Name                     | Unit       | Remark |                                     |
| ------------ | ------------------------ | ---------- | ------ | ----------------------------------- |
| 0            | TCP x                    | meter      |        | {base} frame                        |
| 1            | TCP y                    | meter      | L!=R   | {base} frame                        |
| 2            | TCP z                    | meter      |        | {base} frame                        |
| 3            | EEP x                    | meter      |        | {base} frame                        |
| 4            | EEP y                    | meter      | L!=R   | {base} frame                        |
| 5            | EEP z                    | meter      |        | {base} frame                        |
| 6            | EE Roll                  | **DEGREE** |        | {base}2{tcp} rzyz(yaw, pitch, roll) |
| 7            | EE Pitch                 | **DEGREE** |        | {base}2{tcp} rzyz(yaw, pitch, roll) |
| 8            | EE Yaw: Cr               | **DEGREE** | L!=R   | {base}2{tcp} rzyz(yaw, pitch, roll) |
| 9            | Manipulability           | -          |        |                                     |
| 10           | Joint_0: Waist_Roll      | radian     | L!=R   |                                     |
| 11           | Joint_1: Waist_Pitch     | radian     |        |                                     |
| 12           | Joint_2: RShoulder_Pitch | radian     |        |                                     |
| 13           | Joint_3: RShoulder_Roll  | radian     | L!=R   |                                     |
| 14           | Joint_4: RElbow_Pitch    | radian     |        |                                     |
| 15           | Joint_5: RElbow_Yaw      | radian     | L!=R   |                                     |
| 16           | Joint_6: RWrist_Pitch    | radian     |        |                                     |
| 17           | Joint_7: RWrist_Roll     | radian     | L!=R   |                                     |

"robocare_right_reachability_map.npy" has 18 columns. (Index 17 is "Joint_7".)

# OLD IRM

CURRENT IMPORTANT INFO.
pitch = 90, roll = -90

| Column Index | Name               | Unit   | Remark       |
| ------------ | ------------------ | ------ | ------------ |
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

# NEW IRM

- Sample: ./config/robocare_right_irm.npy
  - This sample has 7899 points.
  - The supported z of target is {0.7, 0.75, 0.8}.

| Column Index | Name                     | Unit       | Remark           |
| ------------ | ------------------------ | ---------- | ---------------- |
| 0            | Mobile Base x            | meter      | For query output |
| 1            | Mobile Base y            | meter      | For query output |
| 2            | Target Object z (height) | meter      | For query input  |
| 3            | EEP x                    | meter      | For IK solver    |
| 4            | EEP y                    | meter      | For IK solver    |
| 5            | EEP z                    | meter      | For IK solver    |
| 6            | EE Roll                  | **DEGREE** |                  |
| 7            | EE Pitch                 | **DEGREE** |                  |
| 8            | EE Yaw                   | **DEGREE** | Cr               |
| 9            | Manipulability           | -          |                  |
| 10           | Joint_0 value            | radian     |                  |
| 11           | Joint_1 value            | radian     |                  |
| >=12         | Joint_2... values        | radian     |                  |
