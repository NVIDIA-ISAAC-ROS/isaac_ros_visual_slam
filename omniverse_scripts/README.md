# Explanation of the start_isaac_sim file to open and play Isaac Sim examples

```
alias omni_python='~/.local/share/ov/pkg/isaac_sim-2022.2.1/python.sh'
omni_python start_isaac_sim_visual_slam.py 
```

#### **Parameters Explained**

| Parameter           | Default                                                                                                        | Description                                                                                                                                                                                                |
|---------------------|------------------------------------------------------------------------------------------------------------------|------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `scenario_path`        | */Isaac/Samples/ROS2/Scenario/carter_warehouse_apriltags_worker.usd*| Path of the scenario to launch relative to the nucleus server base path. Scenario must contain a carter robot.|
| `environment_prim_path` | */World/WareHouse* | Path to the world to create a navigation mesh.|
| `tick_rate_hz` | *20* | The rate (in hz) that we step the simulation at.|
