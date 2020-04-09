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
import csv
from enum import Enum
import rospy


DEFAULT_LOG_DIRECTORY = rospkg.RosPack().get_path('benchmark') + "/log/"
DEFAULT_TXT_FILE_NAME = "log.txt"
DEFAULT_CSV_MAKESPAN_FILE_NAME = "log_makespan.csv"
DEFAULT_CSV_FLOWTIME_FILE_NAME = "log_flowtime.csv"
DEFAULT_CSV_FLOWTIME_AVG_FILE_NAME = "log_flowtime_avg.csv"
DEFAULT_CSV_MAKESPAN_AVG_FILE_NAME = "log_makespan_avg.csv"
DEFAULT_CSV_WPTIME_FILE_NAME = "log_wptime.csv"


class LogValues(Enum):
    """ Enum of log values.
    """
    MAKESPAN = 'makespan'
    FLOWTIME = 'flowtime'
    AVG_FLOWTIME = 'avg_flowtime'
    WPTIME = 'wptime'
    AVG_MAKESPAN = 'avg_makespan'


class FileType(Enum):
    """ Supported log file types.
    """
    TXT = 'TXT'
    CSV = 'CSV'


class Logger:
    """ Logger.
    """

    def __init__(self, log_dir=DEFAULT_LOG_DIRECTORY,
                 initial_setup_print=True,
                 output_to_csv=True,
                 output_to_txt=True):
        """ Init. method.
        :param log_dir:
        :param initial_setup_print:
        :param output_to_csv:
        :param output_to_txt:
        """
        self._log_dir = log_dir
        self._log_file_paths = {
            FileType.TXT.value: log_dir + DEFAULT_TXT_FILE_NAME,
            FileType.CSV.value: {
                LogValues.MAKESPAN.value: log_dir + DEFAULT_CSV_MAKESPAN_FILE_NAME,
                LogValues.FLOWTIME.value: log_dir + DEFAULT_CSV_FLOWTIME_FILE_NAME,
                LogValues.AVG_MAKESPAN.value: log_dir + DEFAULT_CSV_MAKESPAN_AVG_FILE_NAME,
                LogValues.AVG_FLOWTIME.value: log_dir + DEFAULT_CSV_FLOWTIME_AVG_FILE_NAME,
                LogValues.WPTIME.value: log_dir + DEFAULT_CSV_WPTIME_FILE_NAME
            }
        }
        self._output_to_txt = output_to_txt
        self._output_to_csv = output_to_csv
        self._log_file = {}
        self._id = calendar.timegm(time.gmtime())
        if initial_setup_print:
            self.setup()

    def _open(self, file_type, log_value=None):
        """ Opens the log file.
        :param file_type:
        :param log_value:
        """
        if file_type == FileType.TXT or file_type == FileType.TXT.value:
            self._log_file[FileType.TXT.value] = open(
                self._log_file_paths[FileType.TXT.value], 'a')
        elif file_type == FileType.CSV or file_type == FileType.CSV.value:
            self._log_file[FileType.CSV.value] = open(
                self._log_file_paths[FileType.CSV.value][log_value], 'a')

    def _close(self, file_type):
        """ Closes the log file.
        :param file_type:
        """
        if file_type == FileType.TXT or file_type == FileType.TXT.value:
            self._log_file[FileType.TXT.value].close()
        if file_type == FileType.CSV or file_type == FileType.CSV.value:
            self._log_file[FileType.CSV.value].close()

    def _is_opened(self, file_type, log_value=None):
        """ Checks if the log file is ready for writing.
        :param file_type:
        :param log_value:
        :return True, False
        """
        try:
            if log_value is None:
                return not self._log_file[file_type].closed
            return not self._log_file[file_type].closed
        except KeyError:
            return False

    def _write(self, text, log_value=None, timestamp=True):
        """ Writes in all enabled files.
        :param text:
        :param timestamp:
        :param log_value:
        """
        if self._output_to_txt:
            self._write_to_file(text, FileType.TXT, log_value=log_value, timestamp=timestamp)
        elif self._output_to_csv:
            self._write_to_file(text, FileType.CSV, log_value=log_value, timestamp=timestamp)
        else:
            print("No file for logging ...")

    def _write_to_file(self, text, file_type, log_value=None, timestamp=True):
        """ Writes to the log file.
        :param text:
        :param file_type:
        :param timestamp:
        :param log_value:
        """
        if not self._is_opened(file_type, log_value):
            self._open(file_type, log_value)
        if file_type == FileType.TXT or file_type == FileType.TXT.value:
            self._write_to_txt_file(text, timestamp)
        elif file_type == FileType.CSV or file_type == FileType.CSV.value:
            self._write_to_csv_file(text)
        else:
            raise NotImplementedError("Other log file type not supported yet!")
        self._close(file_type)

    def _write_to_csv_file(self, text):
        """ Writes the text to the log csv file.
        :param text: ['col1', 'col2', ...]
        """
        text.insert(0, str(self._id))
        writer = csv.writer(self._log_file[FileType.CSV.value])
        writer.writerow(text)

    def _write_to_txt_file(self, text, timestamp=True):
        """ Writes text to the log file.
        :param text:
        :param timestamp:
        """
        if timestamp:
            text = "[" + str(datetime.datetime.now()) + "] " + text
        self._log_file[FileType.TXT.value].write(text + "\n")

    def info(self, text, quiet=False):
        """ Straightforward log for plain text.
        :param text:
        :param quiet:
        """
        if not quiet:
            rospy.loginfo(text)
        if self._output_to_txt:
            self._write_to_file(text, FileType.TXT.value)
        if self._output_to_csv:
            # CSV log does not support plain text log (unintended); do nothing
            pass

    def wptime(self, robot, wp_1, wp_2, _time, quiet=False):
        """ Logs the time a robot needed from wp 1 to 2.
        :param robot:
        :param wp_1:
        :param wp_2:
        :param _time:
        :param quiet:
        """
        text = "[WPTIME] " + str(robot) + " from " + str(wp_1) + " to " + str(wp_2) + " in " + str(_time)
        if not quiet:
            rospy.loginfo(text)
        if self._output_to_txt:
            self._write_to_file(text, FileType.TXT.value)
        if self._output_to_csv:
            text = [str(robot), str(wp_1), str(wp_2), str(_time)]
            self._write_to_file(text, FileType.CSV.value, log_value=LogValues.WPTIME.value)

    def flowtime(self, robot, _time, quiet=False):
        """ Logs the flowtime, the time a robot needed to get from wp to wp in average.
        :param robot:
        :param _time:
        :param quiet:
        """
        text = "[FLOWTIME] " + str(robot) + ": " + str(_time)
        if not quiet:
            rospy.loginfo(text)
        if self._output_to_txt:
            self._write_to_file(text, FileType.TXT.value)
        if self._output_to_csv:
            text = [str(robot), str(_time)]
            self._write_to_file(text, FileType.CSV.value, log_value=LogValues.FLOWTIME.value)

    def flowtime_avg(self, _time, quiet=False):
        """ Logs the average flowtime, the time all robots needed to get from wp to wp in average.
        :param _time:
        :param quiet:
        """
        text = "[FLOWTIME AVG] " + str(_time)
        if not quiet:
            rospy.loginfo(text)
        if self._output_to_txt:
            self._write_to_file(text, FileType.TXT.value)
        if self._output_to_csv:
            text = [str(_time)]
            self._write_to_file(text, FileType.CSV.value, log_value=LogValues.AVG_FLOWTIME.value)

    def makespan(self, robot, makespan, quiet=False):
        """ Logs the makespan for a robot.
        :param robot:
        :param makespan:
        :param quiet:
        """
        text = "[MAKESPAN] " + str(robot) + " finished in " + str(makespan) + "s"
        if not quiet:
            rospy.loginfo(text)
        if self._output_to_txt:
            self._write_to_file(text, FileType.TXT.value)
        if self._output_to_csv:
            text = [str(robot), str(makespan)]
            self._write_to_file(text, FileType.CSV.value, log_value=LogValues.MAKESPAN.value)

    def makespan_avg(self, makespan, quiet=False):
        """ Logs the makespan average for all robots.
        :param makespan:
        :param quiet:
        """
        text = "[MAKESPAN AVG] " + str(makespan) + "s"
        if not quiet:
            rospy.loginfo(text)
        if self._output_to_txt:
            self._write_to_file(text, FileType.TXT.value)
        if self._output_to_csv:
            text = [str(makespan)]
            self._write_to_file(text, FileType.CSV.value, log_value=LogValues.AVG_MAKESPAN.value)

    def setup(self):
        """ This will read the settings file and print
        all necessary information into the log file.
        """
        sm = settings.SettingsManager()
        text = "\n\n---------------------- BENCHMARK ----------------------\n" \
               + "==> datetime: " + str(datetime.datetime.now()) + "\n" \
               + "==> Benchmark ID: " + str(self._id) + "\n" \
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
               + "==> include_start_time: " + str(sm.read("include_start_time")) + "\n" \
               + "--------------------------------------------------------\n"
        self._write_to_file(text, FileType.TXT.value, timestamp=False)
