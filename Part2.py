# -*- coding: utf-8 -*-
"""
Part 2
Created on Mon Jan 16 16:17:02 2023

@author: wali
"""


from lib.lib_main import * # importing modules from the folder

Simulation_TEND = 200.0 # Simulation lenght in seconds
SimParams = SimParams_(Simulation_TEND)

Open_Loop_Pitch = 7.824 #  pitch angle in degrees
Open_Gen_Torque = 9.95025e6 # generator reaction torque in Nm

Controller_Type = 'P' # Controller Type, OL:Open Loop, P: Porportional
Kp = 1
Controller = Controller_(Controller_Type, Kp = Kp, Open_Loop_Pitch = Open_Loop_Pitch, Open_Gen_Torque  = Open_Gen_Torque)

WT_Model = 'WT0' # Model complexity,  WT0:Rotor,   WT1:Rotor+DT,  WT2:Rotor+DT+Tower fore-aft
WT = WT_(WT_Model,SimParams)

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

data = simulate(SimParams, WT, Controller, wind_profile_options)

figsize = (8,8)
gen_plot(WT, SimParams, data,figsize)