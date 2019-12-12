#!/usr/bin/env python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      2019
@brief:     This logger will store the logs in a txt file.
@todo:
------------------------------------------------------------- """


import rospkg
import datetime


DEFAULT_LOG_DIRECTORY = rospkg.RosPack().get_path('benchmark') + "/log/eval_log.txt"


class Logger:
    """ Logger.
    """

    def __init__(self, log_file=DEFAULT_LOG_DIRECTORY):
        """ Init. method.
        :param log_file:
        """
        self._log_file_path = log_file
        self._log_file = None

    def _open(self):
        """ Opens the log file.
        """
        self._log_file = open(self._log_file_path, 'a')

    def _close(self):
        """ Closes the log file.
        """
        self._log_file.close()

    def _is_opened(self):
        """ Checks if the log file is ready for writing.
        :return True, False
        """
        return not self._log_file.closed

    def _write(self, text, timestamp=True):
        """ Writes text to the log file.
        :param text:
        :param timestamp:
        """
        if type(text) is not str:
            return
        if self._log_file is None or not self._is_opened():
            self._open()
        if timestamp:
            text = "[" + str(datetime.datetime.now()) + "] " + text
        self._log_file.write(text + "\n")
        self._close()

    def info(self, text):
        """ Straightforward log for plain text.
        :param text:
        """
        self._write(text)

    def time(self, robot, wp_1, wp_2, time):
        """ Logs the time a robot needed from wp 1 to 2.
        :param robot:
        :param wp_1:
        :param wp_2:
        :param time:
        """
        text = "[WP TIME] " + str(robot) + " from " + str(wp_1) + " to " + str(wp_2) + " in " + str(time)
        self._write(text)
