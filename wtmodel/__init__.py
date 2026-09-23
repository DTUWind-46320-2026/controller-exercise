"""Initialization file for wtmodel package"""

import warnings
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import linalg
from scipy.interpolate import RegularGridInterpolator


class SimulationParameters:
    """Simulation settings"""

    def __init__(self, sim_tend=200, ts=0.1, omega0=1):
        self.sim_tend = sim_tend  # simulation end time [s]
        self.ts = ts  # Controller sampling time [s]
        self.omega0 = omega0  # initial rotor speed [rad/s]
        self.nsim = np.floor(self.sim_tend / self.ts).astype(
            int
        )  # number of sim. steps [-]


class WindTurbine:
    """Wind turbine parameters"""

    # Parameters in file
    CSV_folder = (
        Path(".") / "WT_Data" / "DTU10MW"
    )  # folder containing CP, CT, pitch_grid, and tsr_grid CSVs

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
        self.tsr_grid = np.loadtxt(self.CSV_folder / "tsr_grid.csv", delimiter=",")
        self.pitch_grid = np.loadtxt(self.CSV_folder / "pitch_grid.csv", delimiter=",")
        self.CP_grid = np.loadtxt(self.CSV_folder / "CP.csv", delimiter=",")
        self.CT_grid = np.loadtxt(self.CSV_folder / "CT.csv", delimiter=",")

        self.tsr_range = self.tsr_grid[:, 0]
        self.pitch_range = self.pitch_grid[0, :]
        self.tsr_min, self.tsr_max = np.min(self.tsr_range), np.max(self.tsr_range)
        self.pitch_min, self.pitch_max = (
            np.min(self.pitch_range),
            np.max(self.pitch_range),
        )

        # Convert to numpy arrays
        # tsr = self.tsr_range.to_numpy()  # TODO clean
        # pitch = self.pitch_range.to_numpy()
        tsr = self.tsr_range
        pitch = self.pitch_range

        # cp_values = self.CP_grid.to_numpy()  # TODO clean
        # ct_values = self.CT_grid.to_numpy()
        cp_values = self.CP_grid
        ct_values = self.CT_grid

        # Check orientation!
        self.Cp = RegularGridInterpolator(
            (tsr, pitch),
            cp_values,
            method="linear",
            bounds_error=False,
            fill_value=None,
        )

        self.Ct = RegularGridInterpolator(
            (tsr, pitch),
            ct_values,
            method="linear",
            bounds_error=False,
            fill_value=None,
        )

        # self.Cp = interpolate.interp2d(
        #     self.tsr_range, self.pitch_range, self.CP_grid.T, kind="linear"
        # )
        # self.Ct = interpolate.interp2d(
        #     self.tsr_range, self.pitch_range, self.CT_grid.T, kind="linear"
        # )

        # ------ get continuous state-space matrices ------

        # WT0: Rotor
        if self.Model == "WT0":
            A_c = np.array([[0.0]])  # Continuous time system
            B_c = np.array([[1.0, -1.0]])
            C_c = np.eye(len(A_c))
            D_c = np.zeros([C_c.shape[0], B_c.shape[1]])
            # # discretise the system  # TODO clean
            # self.A, self.B, self.C, self.D = discretise(A_c, B_c, C_c, D_c, SimParams)

        # WT1: Rotor+DT
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
            # # discretise the system  # TODO clean
            # self.A, self.B, self.C, self.D = discretise(A_c, B_c, C_c, D_c, SimParams)

        # Rotor+DT+Tower fore-aft
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
            # # discretise the system  # TODO clean
            # self.A, self.B, self.C, self.D = discretise(A_c, B_c, C_c, D_c, SimParams)

        else:
            print("Invalid wind turbine model!")

        # discretise the system
        self.A, self.B, self.C, self.D = discretise(A_c, B_c, C_c, D_c, SimParams)


class Controller:
    """Controller parameters"""

    Kk = 11.35
    CornerFreq = 0.25 * 2 * np.pi  # rad/s  low pass filter cut-off frequency
    Pnom = 10e6
    Ng = 50
    GenRot_nom = 9.6 * np.pi / 30  # rad/s
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


def load_wsp_array(sim_params, wind_file_name):
    """Load wind speed array from file"""
    t = np.linspace(0, sim_params.sim_tend, sim_params.nsim)
    wsp_data = np.loadtxt(wind_file_name)
    wsp = np.interp(t, wsp_data[:, 0], wsp_data[:, 1])
    return wsp, t


def WT_nonlinear(x, y, u, wsp, WT):
    """Nonlinear wind turbine model"""
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

    Pe = 0.5 * WT.Air_density * np.pi * WT.Rotor_Radius**2 * ve**3 * WT.Cp((tsr, pitch))
    Qa = Pe / omega
    Ft = (
        -0.5
        * WT.Air_density
        * np.pi
        * WT.Rotor_Radius**2
        * ve**2
        * WT.Ct((tsr, pitch))
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
    """
    PI controller action

    Arguments
    -----------
    GenRot : float, int
        Rotational speed in rad/s.
    SimParams : wtmodel.SimParams_
        Instance of SimParams_ class.
    Controller : wtmodel.Controller_
        Instance of Controller_ class.

    Returns
    ----------
    U : np.ndarray
        Array of shape (2, 1) with elements pitch [deg] and generator torque [Nm].
    """
    if GenRot > 2.5:
        warnings.warn("Rotational speed is too high!")
    elif GenRot < 0.2:
        warnings.warn("Rotational speed is too low!")

    Ts = SimParams.ts
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
        warnings.warn("Invalid Controller Type!")

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
        warnings.warn("Invalid Controller Type!")

    # update storage
    Controller.pitch_prev = U_1
    Controller.genTorq_prev = U_2

    # output
    Controller.states_Ei, Controller.states_genFilter = Ei, GenRot_filter
    U = np.array([[U_1], [U_2]])

    return U


def simulate(SimParams, WT, Controller, wind_file_name):
    """
    Simulate a wind turbine with a given controller, simulation parameters
    and wind profile.

    Returns dictionary with:
        x: state vector
        y: output of dynamical system
        u: control actions (pitch [deg] and generator torque[Nm])
        wsp: wind speed [m/s]
        t: time
    """

    x = np.zeros([len(WT.A), 1])
    y = np.zeros([WT.C.shape[0], 1])
    u = np.zeros([2, 1])  # [pitch; Qg]
    x[0, 0] = SimParams.omega0
    wsp, t = load_wsp_array(SimParams, wind_file_name)

    for k in range(SimParams.nsim):
        x_, y_ = WT_nonlinear(x[:, -1], y[:, -1], u[:, -1], wsp[k], WT)
        x, y = np.hstack((x, x_)), np.hstack((y, y_))
        u_ = PI_Controller(x[0, -1], SimParams, Controller)
        u = np.hstack((u, u_))

    data = {"x": x[:, 1:], "y": y[:, 1:], "u": u[:, 1:], "wsp": wsp, "t": t}
    return data


def gen_plot(WT, SimParams, data, figsize):
    """Plot simulation results"""
    t = np.linspace(0, SimParams.sim_tend, SimParams.nsim)
    OmegaR = data["x"][0, :]
    if WT.Model == "WT0":
        OmegaG = WT.GEARBOX_RATIO * OmegaR
        Vt = np.zeros(OmegaG.shape)
    elif WT.Model == "WT1":
        OmegaG = WT.GEARBOX_RATIO * data["x"][1, :]
        Vt = np.zeros(OmegaG.shape)
    elif WT.Model == "WT2":
        OmegaG = WT.GEARBOX_RATIO * data["x"][1, :]
        Vt = data["x"][4, :]
    else:
        warnings.warn("Invalid WT type!")

    BladePitch = data["u"][0, :]
    GenTq = data["u"][1, :]
    # GenTq = data["u"][1, :] / WT.GEARBOX_RATIO

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

    ax[2, 0].plot(t, GenTq)
    ax[2, 0].set(title="Generator Torque [Nm]", xlabel="Time [s]")
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

    plt.tight_layout()
    plt.show()


def discretise(A_c, B_c, C_c, D_c, SimParams):
    """Discretize a continuous system"""
    Ts = SimParams.ts
    N = np.vstack(
        [np.hstack([A_c, B_c]), np.zeros([B_c.shape[1], len(A_c) + B_c.shape[1]])]
    )
    M = linalg.expm(N * Ts)
    A = M[0 : len(A_c), 0 : len(A_c)]
    B = M[0 : len(A_c), len(A_c) : len(A_c) + B_c.shape[1]]
    C = C_c
    D = D_c
    return A, B, C, D
