#!/usr/bin/env python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      2019
@brief:     This will load the settings from the settings file to
            the parameter server.
@todo:
------------------------------------------------------------- """


import src.settings as settings
import rospy


rospy.init_node("settings", anonymous=True)


manager = settings.SettingsManager()
manager.upload()
rospy.loginfo("Settings uploaded to parameter server!")
