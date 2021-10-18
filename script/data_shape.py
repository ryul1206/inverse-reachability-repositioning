# IRM database
# TODO: This is actually.. RM data index...Change it to IRM...
IDX = {
    "TCP_X": 0,  # mobile X
    "TCP_Y": 1,  # mobile Y
    "TCP_Z": 2,  # target Z
    "EEP_X": 3,
    "EEP_Y": 4,
    "EEP_Z": 5,
    "R": 6,
    "P": 7,
    "Y": 8,
    "M": 9,
    "Joint": 10,
}

OLD_IDX = {
    "Mobile_X": 0,
    "Mobile_Y": 1,
    "Mobile_Z": 2,
    "EEP_X": 3,
    "EEP_Y": 4,
    "EEP_Z": 5,
    "Mobile_Qx": 6,
    "Mobile_Qy": 7,
    "Mobile_Qz": 8,
    "Mobile_Qw": 9,
    "M": 10,
    "Joint": 11,
}

# get_candidates output from InverseReachabilityMap class
wIDX = {
    "Ct": 0,  # Degree
    "Cr": 1,  # Degree
    "Bx": 2,  # Base X
    "By": 3,  # Base Y
    "M": 4,  # Manipulability
    "Tz": 5,  # Target object Z (height)
    "EEPx": 6,  # End-effector x, y, z, r, p, y
    "J": 12,  # Joint 1, 2, ...
}
