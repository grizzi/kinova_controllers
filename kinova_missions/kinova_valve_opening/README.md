# kinova_valve_opening

This package contains the state machine for the valve opening task

### Run the simulation 

On separate terminals:

1. `roslaunch kinova_controllers run_sim.launch`
2. `roslaunch kinova_valve_opening mission.launch simulation:=true`

### Run on the hardware

On separate terminals:

1. `roslaunch kinova_robot robot.launch`
2. `roslaunch kinova_valve_opening components.launch`
3. `roslaunch kinova_valve_opening mission.launch`
