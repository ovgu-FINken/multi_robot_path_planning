#!/usr/bin/env python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      2019
@brief:     This logger will store the logs in a txt file.
@todo:
------------------------------------------------------------- """


import rospkg
import datetime
import src.settings as settings
import calendar
import time


DEFAULT_LOG_DIRECTORY = rospkg.RosPack().get_path('benchmark') + "/log/eval_log.txt"


class Logger:
    """ Logger.
    """

    def __init__(self, log_file=DEFAULT_LOG_DIRECTORY, initial_setup_print=True):
        """ Init. method.
        :param log_file:
        :param initial_setup_print:
        """
        self._log_file_path = log_file
        self._log_file = None
        if initial_setup_print:
            self.setup()

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

    def makespan(self, robot, makespan):
        """ Logs the makespan for a robot.
        :param robot:
        :param makespan:
        """
        text = "[MAKESPAN] " + str(robot) + " finished with makespan of " + str(makespan) + "s <<<"
        self._write(text)

    def makespan_avg(self, makespan):
        """ Logs the makespan average for all robots.
        :param makespan:
        """
        text = "[MAKESPAN AVG] Average makespan: " + str(makespan) + "s <<< <<< <<<"
        self._write(text)

    def setup(self):
        """ This will read the settings file and print
        all necessary information into the log file.
        """
        sm = settings.SettingsManager()
        text = "\n\n---------------------- BENCHMARK ----------------------\n" \
               + "==> datetime: " + str(datetime.datetime.now()) + "\n" \
               + "==> Benchmark ID: " + str(calendar.timegm(time.gmtime())) + "\n" \
               + "==> model_name: " + str(sm.read("model_name")) + "\n" \
               + "==> model_type: " + str(sm.read("model_type")) + "\n" \
               + "==> namespace: " + str(sm.read("namespace")) + "\n" \
               + "==> number_of_robots: " + str(sm.read("number_of_robots")) + "\n" \
               + "==> formation: " + str(sm.read("formation")) + "\n" \
               + "==> position: " + str(sm.read("position")) + "\n" \
               + "==> orientation: " + str(sm.read("orientation")) + "\n" \
               + "==> wp_map: " + str(sm.read("wp_map")) + "\n" \
               + "==> wp_threshold: " + str(sm.read("wp_threshold")) + "\n" \
               + "==> world: " + str(sm.read("world")) + "\n" \
               + "==> rounds: " + str(sm.read("rounds")) + "\n" \
               + "==> end_procedure: " + str(sm.read("end_procedure")) + "\n" \
               + "--------------------------------------------------------\n"
        self._write(text, timestamp=False)
