# Raw Sample Data

This CSV file has 893 points.
The order of each column from CSV is:

1. Tool Center Point(TCP) x
2. Tool Center Point(TCP) y
3. Tool Center Point(TCP) z
4. Yaw == Cr(euler: rzyz)
5. Joint 1
6. Joint 2
7. Joint 3
8. Joint 4
9. Joint 5
10. Joint 6
11. Joint 7
12. Manipulability

### (Sample 1) Robocare Right Arm

![img](Screenshot%20from%202021-01-27%2010-50-54.png)

(1) Raw CSV file

- File: [socialrobot_manip_2021-01-25_22-26-56.csv](socialrobot_manip_2021-01-25_22-26-56.csv)
  - This CSV file has 7899 points.
  - The first row is the title.
- Columns:
  - [0] [meter] Tool Center Point(TCP) x
  - [1] [meter] Tool Center Point(TCP) y
  - [2] [meter] Tool Center Point(TCP) z (0.7, 0.75, 0.8)
  - [3] [meter] End-effector Point x
  - [4] [meter] End-effector Point y
  - [5] [meter] End-effector Point z
  - [6] [deg] Yaw == Cr(euler: rzyz)
  - [7] [deg] Pitch(euler: rzyz)
  - [8] [deg] Roll(euler: rzyz)
  - [9] Manipulability
  - [10] [rad] Joint value: Waist_Roll
  - [11] [rad] Joint value: Waist_Pitch
  - [12] [rad] Joint value: RShoulder_Pitch
  - [13] [rad] Joint value: RShoulder_Roll
  - [14] [rad] Joint value: RElbow_Pitch
  - [15] [rad] Joint value: RElbow_Yaw
  - [16] [rad] Joint value: RWrist_Pitch
  - [17] [rad] Joint value: RWrist_Roll

(2) Raw NPY file

원래는
[rad]**angle** = Cr (idx: 0)
[m] **X** (idx: 1)
[m] **Y** (idx: 2)
. **manipulability** (idx: 3)

- index
  - [0] [rad] yaw(Cr) -> gripper orientation (정수여야 filter칠 수 있음)
  - [1] [rad] pitch -> gripper orientation
  - [2] [rad] roll -> gripper orientation
  - [3] tcp x --(transformed)-> mobile base x
  - [4] tcp y --(transformed)-> mobile base y
  - [5] tcp z -> target object z
  - [6] manipulability
