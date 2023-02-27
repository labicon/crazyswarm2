#!/usr/bin/env python

import numpy as np
from pathlib import Path

import numpy as np

import crazyflie_py 

Z = 1.0
T_GOTO = 2.0


def circle_trajectory(N, theta0, omega, r):
    theta = (theta0 + omega * np.arange(N)) % 2*np.pi
    return np.c_[
        r * np.cos(theta), r * np.sin(theta), np.full_like(theta, Z) 
    ]

def main():
    swarm = crazyflie_py.Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    omega = 0.05 # radians / second
    r = 2.0 # meters
    N = 40 # integer time steps
    n_crazyflies = 2

    theta0s = np.arange(0, 2*np.pi * (1.1 - 1/n_crazyflies), 2 * np.pi/n_crazyflies)
    # theta0s = np.linspace(0, 2*np.pi * (1 - 1/n_crazyflies), n_crazyflies, endpoint=True)
    Xs = np.dstack([circle_trajectory(N, theta0, omega, r) for theta0 in theta0s])
    
    allcfs.takeoff(targetHeight=1.0, duration=5.0)
    timeHelper.sleep(5.0)
    for icf, cf in enumerate(allcfs.crazyflies):
        cf.goTo(Xs[0, :, icf], 0, 5.0)
    timeHelper.sleep(7.0)
   
    for t in range(N):
        for icf, cf in enumerate(allcfs.crazyflies):
            Xi = Xs[t, :, icf]
            print(Xi)
            cf.goTo(Xi, 0, T_GOTO)
            timeHelper.sleepForRate(1 / T_GOTO)

    allcfs.land(targetHeight=0.06, duration=2.0)
    timeHelper.sleep(3.0)


if __name__ == "__main__":
    main()
