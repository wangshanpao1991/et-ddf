#!/usr/bin/env python
from __future__ import division
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter
from scipy.stats import norm as normal
from asset import Asset
from measurements import *
from copy import deepcopy
from error_plotting import *
from etdynamics import *
from copy import deepcopy

from pdb import set_trace

# np.random.seed(1)

DEBUG = False

# Simple simulation
K = 1000
world_dim = 3
num_assets = 2
num_ownship_states = 8
num_states = num_ownship_states * num_assets

RED_ASSET_START = 0

comms_drop_prob = 0.3

def main():

    ######## DEFINE STATE && UNCERTAINTY #############

    # GOOD SIM: Red team 7.5, -25
    # x_truth = np.array([[10,5,np.pi,0,0,0,
    #                     -10,-5,0,0,0,0,
    #                     7.5,-25,np.pi/2,0,0,0]], dtype=np.float64).T
    x_truth = np.array([[0,0,0,0,0,0,0,0,
                        -10,-5,-2,0,0,0,0,0]], dtype=np.float64).T
    P_initial = np.array([[.5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,.5,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,.5,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,.5,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,.5,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,.5,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,2,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1]], dtype=np.float64).T

    ####### DEFINE PROCESS NOISE #######
    q = 0.01
    q_yaw = 0.01

    ####### DEFINE PERCEIVED PROCESS NOISE #######
    Q_perceived0 = np.array([[.001,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,.001,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,.01,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,.01,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,.01,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,.01,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,.01,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,.01,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]], dtype=np.float64).T
    Q_perceived1 = np.array([[.01,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,.01,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,.01,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,.01,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,.001,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,.001,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,.01,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,.01,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                          [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]], dtype=np.float64).T
    ###### DEFINE DYNAMICS #########
    linear_dynamics_status = False

    ########## DEFINE MEAS NOISE ##########
    r_gps = 0.3
    r_gps_yaw = 0.01
    r_gps_perceived = 0.5
    r_gps_yaw_perceived = 0.01

    r_range = 0.1
    r_bearing = 0.04
    r_range_perceived = 0.1
    r_bearing_perceived = 0.04
    

    ########## DEFINE ET DELTAS ##########
    # gps_yaw_delta = 0.3
    # gps_xy_delta = 0.2
    gps_yaw_delta = 0.0
    gps_xy_delta = 0.0

    bearing_delta = 0.0
    range_delta = 0.0

    ########## INITIALIZE ASSETS ##########
    asset_starting = deepcopy(x_truth)
    # asset_starting[2*num_ownship_states,0] = RED_ASSET_START
    # asset_starting[2*num_ownship_states+1,0] = RED_ASSET_START
    asset_list = []
    asset = Asset(0, num_ownship_states, world_dim, asset_starting, P_initial, linear_dynamics_status, red_team=[2])
    asset1 = Asset(1, num_ownship_states, world_dim, asset_starting, P_initial, linear_dynamics_status, red_team=[2])
    asset_list.append(asset); asset_list.append(asset1)

    ########## DATA RECORDING ##########
    x_truth_bag = x_truth_bag = deepcopy(x_truth).reshape(-1,1)
    x_hat_bag0 = deepcopy(asset.main_filter.x_hat).reshape(-1,1)
    p_bag0 = deepcopy(asset.main_filter.P)
    x_hat_bag1 = deepcopy(asset1.main_filter.x_hat).reshape(-1,1)
    p_bag1 = deepcopy(asset1.main_filter.P)
    
    seq = 0
    implicit_update_cnt = 0
    total_num_meas_cnt = 0
    comms_success_cnt = 0

    mf0_xhat = deepcopy(asset.main_filter.x_hat)
    mf0_P = deepcopy(asset.main_filter.P)
    mf1_xhat = deepcopy(asset1.main_filter.x_hat)
    mf1_P = deepcopy(asset1.main_filter.P)
    plotting_bag = [[deepcopy(x_truth), mf0_xhat, mf0_P, mf1_xhat, mf1_P, 0, ""]]

    for k in range(K):
        ########## DEFINE CONTROL INPUT ##########
        u0 = np.array([[0.1, -0.0, np.pi/50]], dtype=np.float64).T # Speed, depth speed, ang velocity
        u1 = np.array([[0.1, 0.0, -np.pi/50]], dtype=np.float64).T
        # u0 = get_control(asset.main_filter.x_hat, 0)
        # u1 = get_control(asset1.main_filter.x_hat, 1)
        # u0 = get_control(x_truth, 0)
        # u1 = get_control(x_truth, 1)
        # u2 = np.array([[0.1, 0]], dtype=np.float64).T

        ########## SIMULATE TRUTH MOTION ##########
        x_truth0 = nonlinear_propagation(x_truth, u0, world_dim, num_ownship_states, 0)
        x_truth0[3,0] = normalize_angle(x_truth0[3,0])
        x_truth1 = nonlinear_propagation(x_truth, u1, world_dim, num_ownship_states, 1)
        x_truth1[num_ownship_states+3,0] = normalize_angle(x_truth1[num_ownship_states+3,0])
        # x_truth2 = nonlinear_propagation(x_truth, u2, world_dim, num_ownship_states, 2)
        # x_truth2[2,0] = normalize_angle(x_truth2[2,0])
        x_truth[:num_ownship_states,0] = x_truth0[:num_ownship_states].ravel()
        x_truth[num_ownship_states:2*num_ownship_states,0] = x_truth1[num_ownship_states:2*num_ownship_states].ravel()
        # x_truth[2*num_ownship_states:3*num_ownship_states,0] = x_truth2[2*num_ownship_states:3*num_ownship_states].ravel()

        
        ########## BAG TRUTH DATA ##########
        x_truth_bag = np.concatenate((x_truth_bag, x_truth.reshape(-1,1)), axis=1)

        ########## ADD NOISE TO TRUTH MOTION ##########
        x_truth_no_noise = deepcopy(x_truth)
        x_truth[0,0] += np.random.normal(0, q)
        x_truth[1,0] += np.random.normal(0, q)
        x_truth[2,0] += np.random.normal(0, q)
        x_truth[3,0] = normalize_angle(x_truth[3,0] + np.random.normal(0, q_yaw))

        x_truth[num_ownship_states,0] += + np.random.normal(0, q)
        x_truth[num_ownship_states+1,0] += + np.random.normal(0, q)
        x_truth[num_ownship_states+2,0] += + np.random.normal(0, q)
        x_truth[num_ownship_states+3,0] = normalize_angle(x_truth[num_ownship_states+3,0] + np.random.normal(0, q_yaw) )
        # x_truth[2*num_ownship_states,0] += + np.random.normal(0, q)
        # x_truth[2*num_ownship_states+1,0] += + np.random.normal(0, q)
        # x_truth[2*num_ownship_states+2,0] += + np.random.normal(0, q_yaw)

        ########## PREDICTION STEP  ##########
        asset.predict(u0, Q_perceived0)
        asset1.predict(u1, Q_perceived1)
        # print("prior:\n" + str(asset1.main_filter.x_hat))

        # print("truth:\n" + str(x_truth_no_noise))
        # print("x_hat0:\n" + str(asset.main_filter.x_hat))
        # print("x_hat1:\n" + str(asset1.main_filter.x_hat))
        # print("---")
        # continue #** CHECK

        ########## GENERATE MEASUREMENTS VALUES ##########
        gpsx0 = x_truth[0,0]
        gpsy0 = x_truth[1,0]
        gpsz0 = x_truth[2,0]
        gpsyaw0 = x_truth[3,0]
        gpsx1 = x_truth[num_ownship_states,0]
        gpsy1 = x_truth[num_ownship_states+1,0]
        gpsz1 = x_truth[num_ownship_states+2,0]
        gpsyaw1 = x_truth[num_ownship_states+3,0]
        # gpsx2 = x_truth[2*num_ownship_states,0]
        # gpsy2 = x_truth[2*num_ownship_states+1,0]
        # gpsyaw2 = x_truth[2*num_ownship_states+2,0]

        # Relative 01 bearing
        # src_x = x_truth[0,0]
        # src_y = x_truth[1,0]
        # src_yaw = x_truth[2,0]
        # other_x = x_truth[num_ownship_states,0]
        # other_y = x_truth[num_ownship_states+1,0]
        # diff_x = other_x - src_x
        # diff_y = other_y - src_y
        # bearing01 = np.arctan2(diff_y, diff_x) - src_yaw

        # # Relative 01 Range
        # src_x = x_truth[0,0]
        # src_y = x_truth[1,0]
        # other_x = x_truth[num_ownship_states,0]
        # other_y = x_truth[num_ownship_states+1,0]
        # diff_x = other_x - src_x
        # diff_y = other_y - src_y
        # range01 = np.sqrt(diff_x**2 + diff_y**2)

        # 01 Relative Range
        src_x = x_truth[0,0]
        src_y = x_truth[1,0]
        src_z = x_truth[2,0]
        other_x = x_truth[num_ownship_states,0]
        other_y = x_truth[num_ownship_states+1,0]
        other_z = x_truth[num_ownship_states+2,0]
        diff_x = other_x - src_x
        diff_y = other_y - src_y
        diff_z = other_z - src_z
        relRange01 = np.linalg.norm([diff_x, diff_y, diff_z])

        # Global 0 Azimuth
        global_gps_src = np.array([[-5,10,0]]).T
        src_x = x_truth[0,0]
        src_y = x_truth[1,0]
        src_yaw = x_truth[3,0]
        other_x = global_gps_src[0]
        other_y = global_gps_src[1]
        diff_x = other_x - src_x
        diff_y = other_y - src_y
        globalazimuth0 = np.arctan2(diff_y, diff_x) - src_yaw

        # Global 0 Range
        src_x = x_truth[0,0]
        src_y = x_truth[1,0]
        src_z = x_truth[2,0]
        other_x = global_gps_src[0]
        other_y = global_gps_src[1]
        other_z = global_gps_src[2]
        diff_x = other_x - src_x
        diff_y = other_y - src_y
        diff_z = other_z - src_z
        globalrange0 = np.linalg.norm([diff_x, diff_y, diff_z])

        # Global 0 Elevation
        src_z = x_truth[2,0]
        other_z = global_gps_src[2]
        diff_z = other_z - src_z
        globalelevation0 = np.arcsin(diff_z / globalrange0)

        # # Global 1 Azimuth
        src_x = x_truth[num_ownship_states,0]
        src_y = x_truth[num_ownship_states+1,0]
        src_yaw = x_truth[num_ownship_states+3,0]
        other_x = global_gps_src[0]
        other_y = global_gps_src[1]
        diff_x = other_x - src_x
        diff_y = other_y - src_y
        globalazimuth1 = np.arctan2(diff_y, diff_x) - src_yaw

        # # Global 1 Range
        src_x = x_truth[num_ownship_states,0]
        src_y = x_truth[num_ownship_states+1,0]
        src_z = x_truth[num_ownship_states+2,0]
        other_x = global_gps_src[0]
        other_y = global_gps_src[1]
        other_z = global_gps_src[2]
        diff_x = other_x - src_x
        diff_y = other_y - src_y
        diff_z = other_z - src_z
        globalrange1 = np.linalg.norm([diff_x, diff_y, diff_z])

        # Global 1 Elevation
        src_z = x_truth[num_ownship_states + 2,0]
        other_z = global_gps_src[2]
        diff_z = other_z - src_z
        globalelevation1 = np.arcsin(diff_z / globalrange1)

        # # 02 Relative Azimuth
        # src_x = x_truth[0,0]
        # src_y = x_truth[1,0]
        # src_yaw = x_truth[2,0]
        # other_x = x_truth[2*num_ownship_states,0]
        # other_y = x_truth[2*num_ownship_states+1,0]
        # diff_x = other_x - src_x
        # diff_y = other_y - src_y
        # relBearing02 = np.arctan2(diff_y, diff_x) - src_yaw

        # # 12 Relative Azimuth
        # src_x = x_truth[num_ownship_states,0]
        # src_y = x_truth[num_ownship_states+1,0]
        # src_yaw = x_truth[num_ownship_states+2,0]
        # other_x = x_truth[2*num_ownship_states,0]
        # other_y = x_truth[2*num_ownship_states+1,0]
        # diff_x = other_x - src_x
        # diff_y = other_y - src_y
        # relBearing12 = np.arctan2(diff_y, diff_x) - src_yaw

        # # 12 Relative Range
        # src_x = x_truth[num_ownship_states,0]
        # src_y = x_truth[num_ownship_states+1,0]
        # other_x = x_truth[2*num_ownship_states,0]
        # other_y = x_truth[2*num_ownship_states+1,0]
        # diff_x = other_x - src_x
        # diff_y = other_y - src_y
        # relRange12 = np.sqrt(diff_x**2 + diff_y**2)

        ########## ADD NOIES TO MEASUREMENTS ##########
        gpsx0 += np.random.normal(0, r_gps)
        gpsy0 += np.random.normal(0, r_gps)
        gpsz0 += np.random.normal(0, r_gps)
        gpsyaw0 += np.random.normal(0, r_gps_yaw)
        gpsx1 += np.random.normal(0, r_gps)
        gpsy1 += np.random.normal(0, r_gps)
        gpsz1 += np.random.normal(0, r_gps)
        gpsyaw1 += np.random.normal(0, r_gps_yaw)
        # gpsx2 += np.random.normal(0, r_gps)
        # gpsy2 += np.random.normal(0, r_gps)
        # gpsyaw2 += np.random.normal(0, r_gps_yaw)
        # gpsyaw2 += np.random.normal(0, r_bearing)
        # bearing01 += np.random.normal(0, r_bearing)
        relRange01 += np.random.normal(0, r_range)
        globalazimuth0 += np.random.normal(0, r_bearing)
        globalelevation0 += np.random.normal(0, r_bearing)
        globalrange0 += np.random.normal(0, r_range)
        globalazimuth1 += np.random.normal(0, r_bearing)
        globalelevation1 += np.random.normal(0, r_bearing)
        globalrange1 += np.random.normal(0, r_range)

        # relBearing02 += np.random.normal(0, r_bearing)
        # relRange02 += np.random.normal(0, r_range)
        # relBearing12 += np.random.normal(0, r_bearing)
        # relRange12 += np.random.normal(0, r_range)

        # STOP, CHECK PERFECT MEASUREMENTS

        ########## INITIALIZE MEASUREMENT TYPES ##########
        gpsx0_meas = GPSx_Explicit(0, gpsx0, r_gps_perceived, gps_xy_delta)
        gpsy0_meas = GPSy_Explicit(0, gpsy0, r_gps_perceived, gps_xy_delta)
        gpsz0_meas = GPSz_Explicit(0, gpsz0, r_gps_perceived, gps_xy_delta)
        gpsyaw0_meas = GPSyaw_Explicit(0, gpsyaw0, r_gps_yaw_perceived, gps_yaw_delta)
        gpsx1_meas = GPSx_Explicit(1, gpsx1, r_gps_perceived, gps_xy_delta)
        gpsy1_meas = GPSy_Explicit(1, gpsy1, r_gps_perceived, gps_xy_delta)
        gpsz1_meas = GPSz_Explicit(1, gpsz1, r_gps_perceived, gps_xy_delta)
        gpsyaw1_meas = GPSyaw_Explicit(1, gpsyaw1, r_gps_yaw_perceived, gps_yaw_delta)
        # gpsx2_meas0 = GPSx_Neighbor_Explicit(0, 2, gpsx2, r_gps_perceived, gps_xy_delta)
        # gpsy2_meas0 = GPSy_Neighbor_Explicit(0, 2, gpsy2, r_gps_perceived, gps_xy_delta)
        # gpsyaw2_meas0 = GPSyaw_Neighbor_Explicit(0,2, gpsyaw2, r_gps_yaw_perceived, gps_yaw_delta)
        # gpsx2_meas1 = GPSx_Neighbor_Explicit(1, 2, gpsx2, r_gps_perceived, gps_xy_delta)
        # gpsy2_meas1 = GPSy_Neighbor_Explicit(1, 2, gpsy2, r_gps_perceived, gps_xy_delta)
        # gpsyaw2_meas1 = GPSyaw_Neighbor_Explicit(1,2, gpsyaw2, r_gps_yaw_perceived, gps_yaw_delta)

        # bearing02_meas = Azimuth_Explicit(0,2, relBearing02, r_bearing_perceived, bearing_delta)
        # range02_meas = Range_Explicit(0, 2, relRange02, r_range_perceived, range_delta)
        # bearing12_meas = Azimuth_Explicit(1,2, relBearing12, r_bearing_perceived, bearing_delta)
        # range12_meas = Range_Explicit(1, 2, relRange12, r_range_perceived, range_delta)
        # bearing01_meas = Azimuth_Explicit(0,1, bearing01, r_bearing_perceived, bearing_delta)
        relRange01_meas = Range_Explicit(0, 1, relRange01, r_range_perceived, range_delta)

        globalazimuth0_meas = AzimuthGlobal_Explicit(0, global_gps_src, globalazimuth0, r_bearing_perceived, 0)
        globalelevation0_meas = ElevationGlobal_Explicit(0, global_gps_src, globalelevation0, r_bearing_perceived, 0)
        globalrange0_meas = RangeGlobal_Explicit(0, global_gps_src, globalrange0, r_range_perceived, 0)
        globalazimuth1_meas = AzimuthGlobal_Explicit(1, global_gps_src, globalazimuth1, r_bearing_perceived, 0)
        globalelevation1_meas = ElevationGlobal_Explicit(1, global_gps_src, globalelevation1, r_bearing_perceived, 0)
        globalrange1_meas = RangeGlobal_Explicit(1, global_gps_src, globalrange1, r_range_perceived, 0)

        ########## ASSETS RECEIVE UNSHAREABLE MEASUREMNTS  ##########
        # asset.receive_meas(gpsx0_meas, shareable=False)
        # asset1.receive_meas(gpsx0_meas, shareable=False)
        # asset1.receive_meas(gpsx1_meas, shareable=False)
        # asset.receive_meas(gpsx2_meas, shareable=False)
        # asset1.receive_meas(gpsx2_measp2, shareable=False)

        # STOP, CHECK Improved estimation of asset by sharing


        ########## ASSETS SHARE MEASUREMENTS  ##########

        # comms_successful = np.random.binomial(1, 1-comms_drop_prob)
        # comms_success_cnt += comms_successful

        # global0_comms = not (k % 3)
        # global1_comms = not ((k+1) % 3)
        # rel01_comms = not ((k+2) % 3)

        # asset0_comms = (global0_comms or rel01_comms) and comms_successful
        # asset1_comms = global1_comms and comms_successful

        # print("asset0 comms: " + str(asset0_comms))
        # print("asset1 comms: " + str(asset1_comms))
        # print("global0 comms: " + str(global0_comms))
        # print("global1 comms: " + str(global1_comms))
        # print("rel01 comms: " + str(rel01_comms))
        asset0_comms = True
        asset1_comms = True

        shared_msgs = ""
        sharing = []
        # sharing.append(asset.receive_meas(gpsx0_meas, shareable=True))
        # sharing.append(asset.receive_meas(gpsy0_meas, shareable=True))
        sharing.append(asset.receive_meas(gpsz0_meas, shareable=True))
        sharing.append(asset.receive_meas(gpsyaw0_meas, shareable=asset0_comms))

        sharing.append(asset.receive_meas(globalelevation0_meas, shareable=True))
        sharing.append(asset.receive_meas(globalrange0_meas, shareable=True))
        sharing.append(asset.receive_meas(globalazimuth0_meas, shareable=True))
        # sharing.append(asset1.receive_meas(gpsx1_meas, shareable=True))
        # sharing.append(asset1.receive_meas(gpsy1_meas, shareable=True))
        sharing.append(asset1.receive_meas(gpsz1_meas, shareable=True))
        sharing.append(asset1.receive_meas(gpsyaw1_meas, shareable=asset1_comms))

        sharing.append(asset1.receive_meas(globalelevation1_meas, shareable=True))
        sharing.append(asset1.receive_meas(globalrange1_meas, shareable=True))
        sharing.append(asset1.receive_meas(globalazimuth1_meas, shareable=True))

        sharing.append(asset.receive_meas(relRange01_meas, shareable=True))

        # diff02_x = x_truth[2*num_ownship_states,0] - x_truth[0,0]
        # diff02_y = x_truth[2*num_ownship_states+1,0] - x_truth[1,0]
        # ang_diff =normalize_angle( np.arctan2(diff02_y, diff02_x) - x_truth[2,0] )
        # if np.linalg.norm([diff02_x, diff02_y]) < 10 and np.abs(ang_diff) < (65 * np.pi/180):
        #     sharing.append(asset.receive_meas(bearing02_meas, shareable=asset0_comms))
        #     sharing.append(asset.receive_meas(range02_meas, shareable=asset0_comms))

        # diff12_x = x_truth[2*num_ownship_states,0] - x_truth[num_ownship_states,0]
        # diff12_y = x_truth[2*num_ownship_states+1,0] - x_truth[num_ownship_states + 1,0]
        # ang_diff =normalize_angle( np.arctan2(diff12_y, diff12_x) - x_truth[num_ownship_states + 2,0] )
        # if np.linalg.norm([diff12_x, diff12_y]) < 10 and np.abs(ang_diff) < (65 * np.pi/180):
        #     sharing.append(asset1.receive_meas(bearing12_meas, shareable=asset1_comms))
        #     sharing.append(asset1.receive_meas(range12_meas, shareable=asset1_comms))
        
        # # Asset 0 passing gps to asset 1
        # if rel01_comms and comms_successful:
        #     sharing.append(asset.receive_meas(bearing01_meas, shareable=True))
        #     sharing.append(asset.receive_meas(range01_meas, shareable=True))
        # elif global0_comms and comms_successful:
        #     sharing.append(asset.receive_meas(globalbearing0_meas, shareable=True))
        #     sharing.append(asset.receive_meas(globalrange0_meas, shareable=True))
        # elif global1_comms and comms_successful:
        #     sharing.append(asset1.receive_meas(globalbearing1_meas, shareable=True))
        #     sharing.append(asset1.receive_meas(globalrange1_meas, shareable=True))

        # Share measurements
        for s in sharing:
            if s == None:
                continue
            i = s.keys()[0]
            if isinstance(s[i], Implicit):
                raise Exception("Shared some shit")
                implicit_update_cnt += 1
            total_num_meas_cnt += 1
            a = asset_list[i]
            meas = s[i]
            a.receive_shared_meas(meas)

            if isinstance(meas, Range_Explicit):
                if meas.measured_asset == 2:
                    shared_msgs += "red, "
                else:
                    shared_msgs += 'rel, '
            elif isinstance(meas, RangeGlobal_Explicit):
                shared_msgs += "glob, "
            elif isinstance(meas, GPSyaw_Explicit):
                shared_msgs += "yaw, "
            # elif isinstance(meas, Implicit) or isinstance(meas, AzimuthGlobal_Explicit):
                # pass
            # else:
                # raise Exception("Unanticipated meas type: " + meas.__class__.__name__)

        ########## CORRECTION STEP ##########
        # if seq > 33:
            # print("x_truth: \n" + str(x_truth))
        asset.correct()
        asset1.correct()

        # if seq > 200:
            # print(asset1.main_filter.x_hat)
            # print(asset1.main_filter.P[2*num_ownship_states:3*num_ownship_states, 2*num_ownship_states:3*num_ownship_states])
        # if seq > 205:
            # break

        # if quit:
        #     print(x_truth)
        #     print(asset.main_filter.x_hat)
        #     print(asset1.main_filter.x_hat)
        #     return

        ########## RECORD FITLER DATA ##########

        # Estimate 0
        x_hat_bag0 = np.concatenate((x_hat_bag0, asset.main_filter.x_hat), axis=1)
        p_bag0 = np.concatenate((p_bag0, asset.main_filter.P), axis=1)
        x_hat_bag1 = np.concatenate((x_hat_bag1, asset1.main_filter.x_hat), axis=1)
        p_bag1 = np.concatenate((p_bag1, asset1.main_filter.P), axis=1)
            

        ########## DEBUG FILTER INPUTS ##########
        if DEBUG:
            if seq > 37:
                print(x_truth)
                print("meas")
                print(gpsx2)
                print(gpsy2)
                print(gpsyaw2)
                print(asset.main_filter.x_hat)
                print("---")
            if abs(x_truth[2*num_ownship_states+2,0] - asset.main_filter.x_hat[2*num_ownship_states+2,0]) > 1:
                print("Error Large detected")
                break
            # set_trace()

        seq += 1
        print(str(seq) + " out of " + str(K))
        print(shared_msgs)
        print('---')

        # Plot bagging
        mf0_xhat = deepcopy(asset.main_filter.x_hat)
        mf0_P = deepcopy(asset.main_filter.P)
        mf1_xhat = deepcopy(asset1.main_filter.x_hat)
        mf1_P = deepcopy(asset1.main_filter.P)
        plotting_bag.append([deepcopy(x_truth), mf0_xhat, mf0_P, mf1_xhat, mf1_P, asset0_comms, shared_msgs])
        # set_trace()

    # STEP CHECK MEAN ESTIMATES ARE DECENT
    # print(x_truth)
    # print(asset.main_filter.x_hat)
    # print(asset1.main_filter.x_hat)
    # plot_truth_data(x_truth_bag, num_ownship_states)
    # plot_data(plotting_bag, num_ownship_states, num_assets, global_gps_src, )

    # print("Percent of msgs sent implicitly: " + str((implicit_update_cnt / total_num_meas_cnt)*100))
    # PLOT ERROR BOUNDS
    plot_error(x_truth_bag, x_hat_bag0, p_bag0, num_ownship_states, 0)
    plot_error(x_truth_bag, x_hat_bag1, p_bag1, num_ownship_states, 1)
    print(x_truth)
    print("x_hat0: \n" + str(asset.main_filter.x_hat))
    print("x_hat1: \n" + str(asset1.main_filter.x_hat))

asset_goal_indices = [1,3]
goal_thresh = 1

def get_control(x_hat, asset_id):
    waypts = [[10,5],[-10,5],[-10,-5],[10,-5]]

    asset_x = x_hat[asset_id*num_ownship_states,0]
    asset_y = x_hat[asset_id*num_ownship_states+1,0]

    pursuing = False
    if x_hat[2*num_ownship_states,0] == RED_ASSET_START:
        waypt = waypts[asset_goal_indices[asset_id]]    
        diff_x = waypt[0] - asset_x
        diff_y = waypt[1] - asset_y
        dist = np.linalg.norm([diff_x, diff_y])
        if dist < goal_thresh:
            asset_goal_indices[asset_id] = (asset_goal_indices[asset_id] + 1) % len(waypts)
    
        new_waypt = waypts[ asset_goal_indices[asset_id] ]
    else:
        new_waypt = x_hat[2*num_ownship_states:2*num_ownship_states+2,0]
        pursuing = True
    asset_yaw = x_hat[asset_id*num_ownship_states+2,0]
    diff_x = new_waypt[0] - asset_x
    diff_y = new_waypt[1] - asset_y
    ang = np.arctan2(diff_y, diff_x)
    ang_err = normalize_angle(ang - asset_yaw)

    ang_vel = ang_err if np.abs(ang_err) < np.pi/10 else (np.pi/10) * np.sign(ang_err)
    dist_err = np.linalg.norm([diff_x, diff_y])
    if pursuing:
        x_dot = 0.05
    else:
        x_dot = 0.2 if dist_err > 2 else 0.05
    return np.array([[x_dot, ang_vel]]).T

def normalize_angle(angle):
    return np.mod( angle + np.pi, 2*np.pi) - np.pi

if __name__ == "__main__":
    main()