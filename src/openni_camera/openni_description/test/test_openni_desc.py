#!/usr/bin/env python
# -*- coding: utf-8 -*-

PKG = 'openni_description'

import os
import subprocess
import unittest

import rospkg
import xacro


class TestOpenniDescription(unittest.TestCase):

    def setUp(self):
        pass

    def tearDown(self):
        True

    def test_urdf_turtlebot(self):
        """
        Check if check_urdf command passes with the urdf that is generated in
        the .test file this test case is called from.
        """
        resulted_urdf_file_relpath = "./sample_kobuki.urdf"
        kobuki_xacro_file_path = rospkg.RosPack().get_path('openni_description') + "/test/model/kobuki_description/sample_kobuki.urdf.xacro"
        self.assertTrue(os.path.isfile(kobuki_xacro_file_path))
        xacro_output_memory = xacro.process_file(kobuki_xacro_file_path)
        xacro_output_file = xacro.open_output(resulted_urdf_file_relpath)
        xacro_output_file.write(xacro_output_memory.toprettyxml(indent='  '))
        xacro_output_file.close()
        self.assertTrue(os.path.isfile(resulted_urdf_file_relpath))
        self.assertEqual(0, subprocess.call(["check_urdf", resulted_urdf_file_relpath]))

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_openni_description', TestOpenniDescription) 
