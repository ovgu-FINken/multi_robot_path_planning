tmux kill-session -t Bug2
tmux kill-session -t benchmark
pkill -SIGKILL gzclient
pkill -SIGKILL gzserver
