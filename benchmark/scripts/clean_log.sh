#!/usr/bin/env bash

#################################################
# @author:  Johann Schmidt
# @date:    2019/20
# @brief:   Cleanes all log files.
# @todo:
#################################################


truncate -s 0 ../log/log.txt
truncate -s 0 ../log/log_flowtime.csv
truncate -s 0 ../log/log_flowtime_avg.csv
truncate -s 0 ../log/log_makespan.csv
truncate -s 0 ../log/log_makespan_avg.csv
truncate -s 0 ../log/log_wptime.csv
