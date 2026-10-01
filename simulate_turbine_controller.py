"""
Simulate a simple model of a wind turbine with a controller.

Things you can vary in the script:
    * Wind turbine model dynamics:
        WP0: only a rigid rotor
        WP1: rigid rotor plus a flexible drivetrain
        WP2: rigid rotor, flexible drivetrain, and tower fore-aft motion
    * Wind turbine controller:
        OL: open-loop controller on both pitch and torque
        OL2: open-loop controller only on pitch
        P: P controller on pitch
        PI: PI controller on pitch
        gs-PI: gain-scheduled PI pitch controller
    * Wind speed time series
        Step-wind: files with "step" in names
        Turbulent wind: files starting with "Kaimal"
        Extreme operating gust: EOG.hh
"""

from pathlib import Path

import wtmodel  # importing modules from the folder

ROOT = Path(__file__).parent

# ----------------- simulation settings -----------------
SIM_TEND = 200.0  # simulation length [s]
sim_settings = wtmodel.SimulationParameters(sim_tend=SIM_TEND)

# ----------------- wind turbine model dynamics -----------------
DYNAMICS_MODEL = "WT0"  # see module docstring for options
wind_turbine = wtmodel.WindTurbine(DYNAMICS_MODEL, sim_settings)

# ----------------- wind turbine controller -----------------
CONTROLLER_TYPE = "OL"  # see module docstring for options
OL_PITCH = 1  #  open-loop pitch angle [deg]
OL_GEN_TORQUE = 1  # open-loop generator torque [Nm]

controller = wtmodel.Controller(
    CONTROLLER_TYPE, Open_Loop_Pitch=OL_PITCH, Open_Gen_Torque=OL_GEN_TORQUE
)

# ----------------- wind time series -----------------
WIND_FILE = (
    ROOT / "WindFiles" / "MATLAB_Generated_Steps.hh"
)  # see module docstring for options


# ----------------- simulate the system response -----------------
data = wtmodel.simulate(sim_settings, wind_turbine, controller, WIND_FILE)

# ----------------- visualize results -----------------
figsize = (8, 6)
wtmodel.gen_plot(wind_turbine, sim_settings, data, figsize)
