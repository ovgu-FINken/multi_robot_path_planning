tmux kill-session -t TangentBug
tmux kill-session -t benchmark
pkill -SIGKILL gzclient
pkill -SIGKILL gzserver
