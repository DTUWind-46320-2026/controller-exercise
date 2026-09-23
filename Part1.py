"""
Simulate a simple model of a wind turbine with a controller.

Possible variations in the script:
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
        Step-wind
        Turbulent wind (Kaimal spectrum)
        Extreme operating gust (EOG)
"""

import wtmodel  # importing modules from the folder

# ----------------- simulation settings -----------------
SIM_TEND = 200.0  # simulation length [s]
sim_settings = wtmodel.SimulationParameters(sim_tend=SIM_TEND)

# ----------------- wind turbine model dynamics -----------------
# WT0:Rotor,   WT1:Rotor + DT,  WT2:Rotor + DT + Tower fore-aft
DYNAMICS_MODEL = "WT0"
wind_turbine = wtmodel.WindTurbine(DYNAMICS_MODEL, sim_settings)

# ----------------- wind turbine controller -----------------
# TODO reset
CONTROLLER_TYPE = "OL"  # see module docstring for options
# Open_Loop_Pitch = 1  #  pitch angle in degrees
# Open_Gen_Torque = 1  # generator reaction torque in Nm
OL_PITCH = 7.824  #  open-loop pitch angle [deg]
OL_GEN_TORQUE = 9.95025e6  # open-loop generator torque [Nm]

controller = wtmodel.Controller(
    CONTROLLER_TYPE, Open_Loop_Pitch=OL_PITCH, Open_Gen_Torque=OL_GEN_TORQUE
)

# ------------------------------------------------------------------------------------
#   Choose the wind speed profile here:
# ------------------------------------------------------------------------------------
# Here you can choose the type of wind speed you'd like to use!
# wind_profile_options = 1
# 1: for step wind speed, use WSP_Profile_Generator to produce wind steps!
# 2: for stochastic wind speed, mean wind speed: 8 m/s
# 3: for stochastic wind speed, mean wind speed: 12 m/s
# 4: for stochastic wind speed, mean wind speed: 15 m/s
# 5: for stochastic wind speed, mean wind speed: 18 m/s
# 6: for stochastic wind speed, mean wind speed: 15 m/s ,no wind shear.
# 7: EOG
# 8: for step wind speed below rated, use WSP_Profile_Generator to produce wind steps!

wind_profile_options = 1

data = wtmodel.simulate(sim_settings, wind_turbine, controller, wind_profile_options)

figsize = (8, 6)
wtmodel.gen_plot(wind_turbine, sim_settings, data, figsize)
