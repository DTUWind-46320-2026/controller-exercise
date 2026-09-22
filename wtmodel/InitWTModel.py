# -*- coding: utf-8 -*-
"""
Created on Thu Jan 12 12:41:18 2023

@author: wali
"""


# Wind Turbine Parameters

import numpy as np
import pandas as pd
from scipy import interpolate, linalg


class WT_:
    # Turbine Parameters
    Efficiency = 1
    Pitch_min = 0
    Pitch_max = 30
    Lambda_min = 3.1931
    Lambda_max = 30.0024
    Air_density = 1.225
    Rotational_speed = 9.6
    omega_min = 5.9970
    Rotor_Radius = 89.2
    Omega = 1.005
    Rotor_Swept_Area = 2.4997e04
    Blades_Number = 3
    GEARBOX_RATIO = 50
    Rotor_Inertia = 0.1051157075e09
    Generator_Inertia = 4.3547e04
    Hub_Inertia = 115926
    Rotary_Inertia = 0.1606586967e09
    Power_Rated = 10e6
    GEN_TORQUE_Not_Normalized = 9.9502e06
    GEN_TORQUE = 0.0465
    Torque_Rated = 9.9502e06
    Torque_Rated_Normalized = 0.0465
    Jt = Rotary_Inertia  # this is the total inertia (Ir + ng**2 Ig)
    Jr_ = 0.1051e09
    Jg_ = 108867500
    DT_K = 867.637e6
    DT_C = 6.215e6

    # --------------------------------------------------------------------------------------------
    #                               FOR SIMPLE TOWER FORE-AFT MODEL

    TFF_f = 0.3  # Tower fore-aft frequency!

    omt = 2 * np.pi * TFF_f  # Frequency of the tower fore-aft mode
    zetat = 0.15  # Damping ratio of the tower fore-aft mode
    Mt = 700e3  # Mass of the tower, check its validity

    # --------------------------------------------------------------------------------------------
    Tower_M = Mt
    Tower_K = Mt * omt**2
    Tower_C = 2 * Mt * omt * zetat

    Rotor_Orientation = "Upwind"
    Rotor_Configuration = "3 Blades"
    Control = "Variable Speed, Collective Pitch"
    Drivetrain = "High Speed, Multiple-Stage Gearbox"
    Rotor_Diameter = 178.3000
    Hub_Diameter = 5.6
    Hub_Height = 115.00
    Cut_In = 4
    Rated_WSP = 11.4000
    Cut_Out = 25
    Cut_In_Rotor_Speed = 6.9000
    Rated_Tip_Speed = 90
    Overhang = 7.1
    Shaft_Tilt = 5
    Precone = 2.5000
    Rotor_Mass = 110000
    Nacelle_Mass = 240000
    Tower_Mass = 347460

    def __init__(self, Model, SimParams):
        self.Model = Model
        # load cp table
        self.tsr_grid = pd.read_csv("WT_Data/DTU10MW/tsr_grid.csv")
        self.pitch_grid = pd.read_csv("WT_Data/DTU10MW/pitch_grid.csv")
        self.CP_grid = pd.read_csv("WT_Data/DTU10MW/CP.csv")
        self.CT_grid = pd.read_csv("WT_Data/DTU10MW/CT.csv")

        self.tsr_range = self.tsr_grid.iloc[:, 0]
        self.pitch_range = self.pitch_grid.iloc[0, :]
        self.tsr_min, self.tsr_max = np.min(self.tsr_range), np.max(self.tsr_range)
        self.pitch_min, self.pitch_max = (
            np.min(self.pitch_range),
            np.max(self.pitch_range),
        )

        self.Cp = interpolate.interp2d(
            self.tsr_range, self.pitch_range, self.CP_grid.T, kind="linear"
        )
        self.Ct = interpolate.interp2d(
            self.tsr_range, self.pitch_range, self.CT_grid.T, kind="linear"
        )
        if self.Model == "WT0":
            A_c = np.array([[0.0]])  # Continuous time system
            B_c = np.array([[1.0, -1.0]])
            C_c = np.eye(len(A_c))
            D_c = np.zeros([C_c.shape[0], B_c.shape[1]])
            # discretise the system
            self.A, self.B, self.C, self.D = discretise(A_c, B_c, C_c, D_c, SimParams)

        elif self.Model == "WT1":
            A_c = np.array(
                [
                    [
                        -self.DT_C / self.Jr_,
                        self.DT_C / self.Jr_,
                        -self.DT_K / self.Jr_,
                    ],
                    [self.DT_C / self.Jg_, -self.DT_C / self.Jg_, self.DT_K / self.Jg_],
                    [1, -1, 0],
                ]
            )
            B_c = np.array([[1, 0], [0, -1], [0, 0]])
            C_c = np.array([[0, 1, 0]])
            D_c = np.zeros([C_c.shape[0], B_c.shape[1]])
            # discretise the system
            self.A, self.B, self.C, self.D = discretise(A_c, B_c, C_c, D_c, SimParams)

        elif self.Model == "WT2":
            A_c = np.array(
                [
                    [
                        -self.DT_C / self.Jr_,
                        self.DT_C / self.Jr_,
                        -self.DT_K / self.Jr_,
                        0,
                        0,
                    ],
                    [
                        self.DT_C / self.Jg_,
                        -self.DT_C / self.Jg_,
                        self.DT_K / self.Jg_,
                        0,
                        0,
                    ],
                    [1, -1, 0, 0, 0],
                    [0, 0, 0, 0, 1],
                    [
                        0,
                        0,
                        0,
                        -self.Tower_K / self.Tower_M,
                        -self.Tower_C / self.Tower_M,
                    ],
                ]
            )
            B_c = np.array([[1, 0, 0], [0, -1, 0], [0, 0, 0], [0, 0, 0], [0, 0, 1]])
            C_c = np.array([[0, 1, 0, 0, 0]])
            D_c = np.zeros([C_c.shape[0], B_c.shape[1]])
            # discretise the system
            self.A, self.B, self.C, self.D = discretise(A_c, B_c, C_c, D_c, SimParams)

        else:
            print("Invalid wind turbine model!")


def discretise(A_c, B_c, C_c, D_c, SimParams):
    # discretise the system
    Ts = SimParams.Ts
    N = np.vstack(
        [np.hstack([A_c, B_c]), np.zeros([B_c.shape[1], len(A_c) + B_c.shape[1]])]
    )
    M = linalg.expm(N * Ts)
    A = M[0 : len(A_c), 0 : len(A_c)]
    B = M[0 : len(A_c), len(A_c) : len(A_c) + B_c.shape[1]]
    C = C_c
    D = D_c
    return A, B, C, D
