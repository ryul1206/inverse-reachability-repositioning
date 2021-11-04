#!/usr/bin/env python
import numpy as np
import rospy
from vision_msgs.msg import BoundingBox2D
from irm_server.srv import Repositioning, RepositioningRequest
"""
[SERVICE RUN]
roslaunch irm_server irm_server.launch

[REQUEST EXAMPLE]
rosrun irm_server request.py
"""


def request():
    req = RepositioningRequest()
    req.hand_type = req.RIGHT_HAND

    table = BoundingBox2D()
    table.center.x = 1.0
    table.center.y = 0.5
    table.center.theta = np.radians(0.0)
    table.size_x = 2.0
    table.size_y = 1.0

    cup = BoundingBox2D()
    cup.center.theta = np.radians(0.0)
    cup.size_x = 0.07
    cup.size_y = 0.07

    # IRM Config ######################3
    # req.Cr.x = np.radians(-90)
    # req.Cr.y = np.radians(90)
    # req.Ct.x = np.radians(-180)
    # req.Ct.y = np.radians(180)

    # req.Cr.x = np.radians(-90)
    # req.Cr.y = np.radians(90)
    # req.Ct.x = np.radians(-5)
    # req.Ct.y = np.radians(5)

    """
    old(500): avg=1.18063844442, std=0.31326705585
    new(500): avg=0.249696707249, std=0.0254426926896
    """
    req.Cr.x = np.radians(-2)
    req.Cr.y = np.radians(2)
    req.Ct.x = np.radians(-180)
    req.Ct.y = np.radians(180)

    # """
    # old
    # new(500):

    # """
    # req.Cr.x = np.radians(-2)
    # req.Cr.y = np.radians(2)
    # req.Ct.x = np.radians(-135.0)
    # req.Ct.y = np.radians(135.0)

    # """
    # old(500): avg=0.695025597095, std=0.156639099771
    # new(500): avg=0.13861464262, std=0.015473369404
    # """
    # req.Cr.x = np.radians(-2)
    # req.Cr.y = np.radians(2)
    # req.Ct.x = np.radians(-90.0)
    # req.Ct.y = np.radians(90.0)
    #########################

    # """
    # old
    # new(500): avg=0.0824957971573, std=0.0114825843096
    # """
    # req.Cr.x = np.radians(-2)
    # req.Cr.y = np.radians(2)
    # req.Ct.x = np.radians(-45.0)
    # req.Ct.y = np.radians(45.0)
    # #####################################

    req.section_definition.x = 0.05
    req.section_definition.y = 1.0
    req.section_definition.z = 0.02
    req.max_dist = 1024.0
    req.collision_offset = 0.3  # IMPORTANT
    req.strict_dual = False

    rospy.wait_for_service("/new_irm_server/find_positions")
    try:
        feasi_proxy = rospy.ServiceProxy("/new_irm_server/find_positions", Repositioning)

        ####################
        import time

        iteration = 500

        com = "new"
        # com = "old"

        req.style = "%s_irm" % com
        seconds = []
        for _ in xrange(iteration):
            print(_)
            cup.center.x = np.random.uniform(0.0, 2.0)
            cup.center.y = np.random.uniform(0.0, 1.0)
            req.Pt.x = cup.center.x
            req.Pt.y = cup.center.y
            req.Pt.z = np.random.uniform(0.75, 0.8)  #0.75  # 0.75, 0.8
            req.Obs = [table, cup]

            start = time.time()
            resp = feasi_proxy(req)
            duration = time.time() - start
            seconds.append(duration)

        avg = np.average(seconds)
        stddev = np.std(seconds)
        print("%s ===========" % com)
        print(avg)
        print(stddev)

        # Save to CSV
        import os
        import datetime
        import csv

        file_name = "timetest_500_180{}".format(com)
        dir_name = "/home/hr/moveit_ws/src/inverse-reachability-repositioning/example"
        ymd_hms = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        with open(os.path.join(dir_name, "{}_{}.csv".format(file_name, ymd_hms)), "w") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["average", avg])
            writer.writerow(["stddev", stddev])
            writer.writerow(["== TYPE ==", com])
            idx = 0
            for sec in seconds:
                writer.writerow([idx, sec])
                idx += 1
        ####################

        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    resp = request()
    # print(resp)