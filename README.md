# Sluggers Simulator

This is a lightweight simulator for testing robot state-control logic for
ECE-118/218 "Sluggers of the Lost Goal".

## Run

```powershell
python sluggers_sim.py
```

Use a repeatable field:

```powershell
python sluggers_sim.py --seed 118
```

Run with the example controller:

```powershell
python sluggers_sim.py --controller sluggers_controller_template.py
```

Run with the ES_Framework-style C adapter:

```powershell
python es_sim_adapter/build_es_adapter.py --project-root ../ece118_finalproject
python sluggers_sim.py --controller es_framework_controller.py
```

Save sensor/control data when the window closes:

```powershell
python sluggers_sim.py --controller sluggers_controller_template.py --log sim_log.jsonl
```

## Controls

- Arrow keys: manual drive
- Space: shoot
- `r`: reset field
- `p`: pause
- `d`: toggle demo/manual mode
- `1`, `2`, `3`: switch dashboard tabs
- Reset button: reset field from the dashboard

## Controller API

Create a Python file with:

```python
def update(sensors, dt):
    return {"vx": 8.0, "vy": 0.0, "omega": 0.0, "shoot": False}
```

For the 4-omni drivetrain, `vx` is forward inches per second, `vy` is left
strafe inches per second, and `omega` is counter-clockwise yaw rate in
radians per second. Legacy tank commands using `left` and `right` still work.
The simulator calls `update()` around 40 times per second.

## ES_Framework C Adapter

The `es_sim_adapter/` folder contains a host-compiled C adapter for testing
state-machine code from the simulator. The Python shim in
`es_framework_controller.py` packs the simulator sensor dictionary into C
structs, calls `SimRobot_Step()`, and returns omni controls to the simulator.

Key files:

- `es_sim_adapter/sim_robot_adapter.h`: sensor/control ABI shared by Python and C
- `es_sim_adapter/sim_robot_adapter.c`: bounded ES-style event loop for each sim tick
- `es_sim_adapter/sim_robot_service.c`: replaceable robot service/HSM scaffold
- `es_sim_adapter/host_stubs/`: PC replacements for `BOARD`, `serial`, and PIC headers
- `es_sim_adapter/build_es_adapter.py`: builds the shared library

To plug in team code later, keep the exported `SimRobot_*` functions stable and
replace or call into `sim_robot_service.c` from your own ES service/HSM files.

The `sensors` dictionary includes:

- `tape`: front/mid/rear tape sensor array plus front/rear/left/right aliases
- `bump`: outside front/rear/left/right bump state
- `track_wire`: left/right obstacle-wire strength approximation
- `ir`: target beacon and obstacle IR range/bearing
- `ping`: ray distance estimates
- `game`: score, ammo, legal zone, hit/goal/disqualification state
- `pose`: simulated robot pose for debugging

## Notes

The field is modeled as 96 in wide by 48 in tall, with a 6 in gap on the left
and right before the black boundary. The black boundary is centered vertically
and is 36 in tall. The robot and enemy beacon spawn on the inner side of their
fields. The 12 in wide ISZ is on the opposite outer end. Boundary tape can be
touched; the robot is only DQ'd when more than half of it leaves the black
boundary's outside edge. The robot body is modeled as 11 in long by 10 in wide.
Obstacles are placed 6 in before each tape offshoot from the spawn side.
Obstacle contact is DQ'd after 1 second.

The field model is approximate because the project PDF does not include exact
measured tape/zone placement values. Everything important is kept in inches near
the top of `sluggers_sim.py`, so you can tune the field constants to match the
real lab setup or your friend's simulator.
