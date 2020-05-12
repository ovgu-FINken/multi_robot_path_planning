tmux kill-session -t Bug2
tmux kill-session -t Simulation_Bug2
tmux kill-session -t benchmark
pkill -SIGKILL gzclient
pkill -SIGKILL gzserver
