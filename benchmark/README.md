# Path Planning Benchmark

## Settings

All adjustable parameter can be found in the settings file: `settings/settings.json`:
```
{
  "model_name": "turtlebot3",
  "model_type": "burger",
  "namespace": "tb3_",
  "number_of_robots": 4,
  "formation": "dense_block",
  "position": [1.5, 0.5, 0.5],
  "orientation": [0.0, 0.0, 0.0]
}
```
Furthermore, RQT and RVIZ can be started by enabling
the `ÈNABLE_RQT` and `ENABLE_RVIZ` flags, respectively.
These flags can be found in the `scripts/benchmark.sh` file.

## Execution

First, make sure you are in the benchmark package.
The benchmark can be executed via:
```
bash ./scripts/benchmark.sh
```
The framework can be closed by typing:
```
bash ./scripts/kill.sh
```
This process can be highly accelerated by creating system keyboard shortcuts like:
```
gnome-terminal -- bash <FULL_PATH>/benchmark.sh
gnome-terminal -- bash <FULL_PATH>/kill.sh
```
