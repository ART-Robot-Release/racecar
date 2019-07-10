#! /usr/bin/env python
import roslib; roslib.load_manifest('openni_launch')

import rospy
from sensor_msgs.msg import PointCloud2
import os
import subprocess
import shutil
import shlex
import tempfile
import signal
import time

def callback(data, args):
    args[0] = True

rospy.init_node('test_sub')

attempt = 0
successes = 0
failures = 0
consecutive_failures = 0
while not rospy.is_shutdown() and consecutive_failures < 5:
    # Bind received flag and use new topic each time to be very sure results
    # aren't affected by tardy callbacks
    received = [False]
    ns = "depth_reg_%d" % attempt
    topic = "/camera/%s/points" % ns
    attempt = attempt + 1
    sub = rospy.Subscriber(topic, PointCloud2, callback, callback_args=received)

    command = "roslaunch openni_launch openni.launch depth_registered:=%s" % ns
    #tmp = tempfile.TemporaryFile(mode='r+')
    tmp = open(os.devnull, 'w')
    p = subprocess.Popen(shlex.split(command), stdout=tmp, stderr=subprocess.STDOUT)
    start = time.time()
    end = start + 15.0
    while (not rospy.is_shutdown()) and (time.time() < end) and (not received[0]):
        time.sleep(0.1)
    duration = time.time() - start

    sub.unregister()
    p.send_signal(signal.SIGINT) # roslaunch will bring everything down
    p.wait()

    if not rospy.is_shutdown():
        if received[0]:
            rospy.loginfo("Succeeded in %.1fs", duration)
            successes = successes + 1
            consecutive_failures = 0
            status = "success"
        else:
            rospy.logerr("Failed!")
            failures = failures + 1
            consecutive_failures = consecutive_failures + 1
            status = "fail"
        if 0:
            # Write out log
            with open("%s_%05d.log" % (status, attempt), 'w') as f:
                tmp.seek(0)
                shutil.copyfileobj(tmp, f)
            # Move core dump if present (would be at ~/.ros/core)
            try:
                core_name = "core_%d" % attempt
                os.rename("core", core_name)
                print("Core dump %s" % core_name)
            except OSError:
                pass

print("Successes: %d" % successes)
print("Failures: %d" % failures)
