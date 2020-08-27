tmux kill-session -t vo_based_algo
tmux kill-session -t benchmark
pkill -SIGKILL gzclient
pkill -SIGKILL gzserver
