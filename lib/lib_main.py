# -*- coding: utf-8 -*-
"""
Created on Tue Jan 10 11:44:49 2023

@author: wali
"""

import warnings

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import interpolate, linalg

from lib.InitWTModel import *

# Simulation Parameters


class SimParams_:
    def __init__(self, Simulation_TEND=200, Ts=0.1):
        self.Simulation_TEND = Simulation_TEND
        self.Ts = Ts  # Controller sampling time [s]
        self.NSim = np.floor(self.Simulation_TEND / self.Ts).astype(int)


class Controller_:
    Kk = 11.35
    CornerFreq = 0.25 * 2 * np.pi  # rad/s  low pass filter cut-off frequency
    Pnom = 10e6
    Ng = 50
    GenRot_nom = 9.6 * np.pi / 30
    Qgmax = 14.9253e6
    Qgmin = 0
    dQgmax = 15e5
    dQgmin = -15e5
    th_max = 90  # deg
    th_min = 0
    dthmax = 8  # deg/s
    dthmin = -8

    states_Ei = 0
    states_genFilter = 0

    def __init__(
        self, Type, Kp=0.75, Ki=0.3, KII=0.1253e8, Open_Loop_Pitch=0, Open_Gen_Torque=0
    ):
        self.Type = Type
        self.Kp = Kp
        self.Ki = Ki
        self.KII = KII
        self.Open_Loop_Pitch = Open_Loop_Pitch
        self.Open_Gen_Torque = Open_Gen_Torque
        self.pitch_prev = Open_Loop_Pitch
        self.genTorq_prev = Open_Gen_Torque


def wind_profile_to_name(argument):
    switcher = {
        1: "MATLAB_Generated_Steps.hh",
        2: "Kaimal_8ms.hh",
        3: "Kaimal_12ms.hh",
        4: "Kaimal_15ms.hh",
        5: "Kaimal_18ms.hh",
        6: "Kaimal_15ms_no_shear.hh",
        7: "EOG.hh",
        8: "MATLAB_Generated_Steps_partial_load.hh",
        9: "step.hh",
    }
    return switcher.get(argument, "Invalid wind profile number!")


def MakeWSP(SimParams, wind_profile_options):
    wsp_file_name = wind_profile_to_name(wind_profile_options)

    t = np.linspace(0, SimParams.Simulation_TEND, SimParams.NSim)
    wsp_data = pd.read_csv("WindFiles/" + wsp_file_name, delim_whitespace=True)
    wsp = np.interp(t, wsp_data.iloc[:, 0], wsp_data.iloc[:, 1])
    return wsp, t


def WT_nonlinear(x, y, u, wsp, WT):
    # reshape (ensuring the size is right)
    x, y, u = x.reshape((len(x), 1)), y.reshape((len(y), 1)), u.reshape((len(u), 1))

    Qg = u[1]
    if WT.Model == "WT2":
        xt2 = x[-1]
        ve = wsp - xt2
    else:
        ve = wsp

    pitch = u[0]
    omega = x[0]
    pitch = min(max(pitch, WT.pitch_min), WT.pitch_max)
    tsr = omega * WT.Rotor_Radius / ve
    tsr = min(max(tsr, WT.tsr_min), WT.tsr_max)

    Pe = 0.5 * WT.Air_density * np.pi * WT.Rotor_Radius**2 * ve**3 * WT.Cp(tsr, pitch)
    Qa = Pe / omega
    Ft = (
        -0.5
        * WT.Air_density
        * np.pi
        * WT.Rotor_Radius**2
        * ve**2
        * WT.Ct(tsr, pitch)
        / WT.Tower_M
    )

    if WT.Model == "WT0":
        U_ = np.array([Qa / WT.Jt, Qg / WT.Jt])
    elif WT.Model == "WT1":
        U_ = np.array([Qa / WT.Jr_, Qg / WT.Jg_])
    elif WT.Model == "WT2":
        U_ = np.array([Qa / WT.Jr_, Qg / WT.Jg_, Ft])
    else:
        print("Invalid WT Model !")

    x = np.dot(WT.A, x) + np.dot(WT.B, U_)
    y = np.dot(WT.C, x) + np.dot(WT.D, U_)

    return x, y


def PI_Controller(GenRot, SimParams, Controller):
    if GenRot > 2.5:
        warnings.warn("Rotational speed is too high!")
    elif GenRot < 0.2:
        warnings.warn("Rotational speed is too low!")

    Ts = SimParams.Ts
    Ei = Controller.states_Ei
    GenRot_filter = Controller.states_genFilter

    # Low-pass filter for generator speed
    Alpha = np.exp((-Ts) * Controller.CornerFreq)
    GenRot_filter = (1 - Alpha) * GenRot + Alpha * GenRot_filter
    # Speed error
    Ep = GenRot_filter - Controller.GenRot_nom

    # Integrated speed error
    Ei = Ei + Ep * Ts

    # Gain scheduling
    Kgs = (
        1
        / (
            1
            + (np.pi / 180.0 * (Controller.pitch_prev - Controller.th_min))
            / Controller.Kk
        )
        * 180
        / np.pi
    )

    # Limit of generator speed where optimal tracking ends
    GenRot_high = Controller.GenRot_nom * 0.95

    # Generator torque control
    Qh = Controller.KII * GenRot_high**2
    Qnom = Controller.Pnom / Controller.GenRot_nom

    if Controller.pitch_prev <= Controller.th_min + 0.1:  # below rated wind speed
        if GenRot_filter <= GenRot_high:
            U_2 = Controller.KII * GenRot_filter**2
        else:
            U_2 = Qh + (Qnom - Qh) * (GenRot_filter - GenRot_high) / (
                Controller.GenRot_nom - GenRot_high
            )
    else:  # Above rated wind speed
        U_2 = Controller.Pnom / GenRot_filter

    # Pitch Control
    if Controller.Type == "OL":
        U_1 = Controller.Open_Loop_Pitch
        U_2 = Controller.Open_Gen_Torque
    elif Controller.Type == "OL2":
        U_1 = Controller.Open_Loop_Pitch
    elif Controller.Type == "P":
        U_1 = Controller.Kp * Ep + Controller.Open_Loop_Pitch
    elif Controller.Type == "PI":
        U_1 = (
            50 * Controller.Kp * Ep
            + 50 * Controller.Ki * Ei
            + Controller.Open_Loop_Pitch
        )  # Kgs ~= 50 at the example wind speed
    elif Controller.Type == "gs-PI":
        U_1 = Kgs * (Controller.Kp * Ep + Controller.Ki * Ei)
    else:
        warninings.warn("Invalid Controller Type!")

    # Saturation limits of control signals
    U_1 = min(max(U_1, Controller.th_min), Controller.th_max)
    U_2 = min(max(U_2, Controller.Qgmin), Controller.Qgmax)
    dU_1 = min(
        max((U_1 - Controller.pitch_prev) / Ts, Controller.dthmin), Controller.dthmax
    )
    dU_2 = min(
        max((U_2 - Controller.genTorq_prev) / Ts, Controller.dQgmin), Controller.dQgmax
    )

    # calculate control input using increments
    U_1 = Controller.pitch_prev + dU_1 * Ts
    U_2 = Controller.genTorq_prev + dU_2 * Ts

    # anti-windup
    if Controller.Type == "OL":
        Ei = 0
    elif Controller.Type == "OL2":
        Ei = 0
    elif Controller.Type == "P":
        Ei = 0
    elif Controller.Type == "PI":
        Ei = (U_1 - 50 * Controller.Kp * Ep - Controller.Open_Loop_Pitch) / (
            50 * Controller.Ki
        )
    elif Controller.Type == "gs-PI":
        Ei = (U_1 - Kgs * Controller.Kp * Ep) / (Kgs * Controller.Ki)
    else:
        warninings.warn("Invalid Controller Type!")

    # update storage
    Controller.pitch_prev = U_1
    Controller.genTorq_prev = U_2

    # output
    Controller.states_Ei, Controller.states_genFilter = Ei, GenRot_filter
    U = np.array([[U_1], [U_2]])

    return U


def simulate(SimParams, WT, Controller, wind_profile_options):
    #
    OmegaInit = 1  # rad/s

    x = np.zeros([len(WT.A), 1])
    y = np.zeros([WT.C.shape[0], 1])
    u = np.zeros([2, 1])  # [pitch;Qg]
    x[0, 0] = OmegaInit
    wsp, t = MakeWSP(SimParams, wind_profile_options)

    for k in range(0, SimParams.NSim):
        x_, y_ = WT_nonlinear(x[:, -1], y[:, -1], u[:, -1], wsp[k], WT)
        x, y = np.hstack((x, x_)), np.hstack((y, y_))
        u_ = PI_Controller(x[0, -1], SimParams, Controller)
        u = np.hstack((u, u_))

    data = {"x": x[:, 1:], "y": y[:, 1:], "u": u[:, 1:], "wsp": wsp, "t": t}
    return data


def gen_plot(WT, SimParams, data, figsize):
    t = np.linspace(0, SimParams.Simulation_TEND, SimParams.NSim)
    OmegaR = data["x"][0, :]
    if WT.Model == "WT0":
        OmegaG = WT.GEARBOX_RATIO * OmegaR
        Vt = np.zeros(OmegaG.shape)
    elif WT.Model == "WT1":
        OmegaG = WT.GEARBOX_RATIO * data["x"][1, :]
        Vt = np.zeors(OmegaG.shape)
    elif WT.Model == "WT2":
        OmegaG = WT.GEARBOX_RATIO * data["x"][1, :]
        Vt = data["x"][4, :]
    else:
        warnings.warn("Invalid WT type!")

    BladePitch = data["u"][0, :]
    GenTq = data["u"][1, :] / WT.GEARBOX_RATIO

    WSP = data["wsp"]
    Pe = OmegaG * GenTq
    Cp = (Pe / WT.Efficiency) / (0.5 * WT.Air_density * WT.Rotor_Swept_Area * WSP**3)

    fig, ax = plt.subplots(4, 2, figsize=figsize)
    fig.tight_layout()
    fig.subplots_adjust(hspace=0.5)
    ax[0, 0].plot(t, OmegaR)
    ax[0, 0].set(title="Rotor Speed [rad/s]", xlabel="Time [s]")
    ax[0, 0].grid()
    ax[0, 0].plot(t, 1.005 * np.ones(t.shape), "r--")

    ax[0, 1].plot(t, OmegaG)
    ax[0, 1].set(title="Generator Speed HSS  [rad/s]", xlabel="Time [s]")
    ax[0, 1].grid()

    ax[1, 0].plot(t, Vt)
    ax[1, 0].set(title="Tower fore-aft velocity [m/s]", xlabel="Time [s]")
    ax[1, 0].grid()

    ax[1, 1].plot(t, BladePitch)
    ax[1, 1].set(title="Blade Pitch [deg]", xlabel="Time [s]")
    ax[1, 1].grid()

    ax[2, 0].plot(t, GenTq * 1e-3)
    ax[2, 0].set(title="Generator Torque [kNm]", xlabel="Time [s]")
    ax[2, 0].grid()

    ax[2, 1].plot(t, WSP)
    ax[2, 1].set(title="Wind Speed [m/s]", xlabel="Time [s]")
    ax[2, 1].grid()

    ax[3, 0].plot(t, Pe)
    ax[3, 0].set(title="Generated Power [MW]", xlabel="Time [s]")
    ax[3, 0].grid()

    ax[3, 1].plot(t, Cp)
    ax[3, 1].set(title="Cp [-]", xlabel="Time [s]")
    ax[3, 1].grid()

    fig2, ax2 = plt.subplots(figsize=(8, 5))
    CP_NN = WT.CP_grid
    CP_NN[CP_NN < 0] = 0
    CS = ax2.contourf(
        WT.pitch_range, WT.tsr_range, CP_NN, 30
    )  # [WT.tsr_range,WT.pitch_range,],
    # ax2.set_ylim(2,18)
    ax2.set_xlabel("Pitch [deg]")
    ax2.set_ylabel("Tip-Speed Ratio [-]")
    cbar = plt.colorbar(CS)
    cbar.ax.set_ylabel("Cp [-]")
    Lambda = WT.Rotor_Radius * OmegaR / WSP
    ax2.plot(BladePitch, Lambda, "r.")
    plt.ylim(2, 18)
    plt.show()
