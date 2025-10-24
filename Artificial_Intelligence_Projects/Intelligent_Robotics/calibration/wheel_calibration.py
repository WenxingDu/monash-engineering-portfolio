# for computing the wheel calibration parameters
import numpy as np
import os
import sys
sys.path.insert(0, "../util")
from util.pibot import PenguinPi


def driveMeters(meters, scale):
    travelVel = 50 #ticks
    # calibratedVel = travelVel*scale
    
    ppi.set_velocity([1,0], tick = travelVel, time = meters/0.097)



def calibrateWheelRadius():
    # Compute the robot scale parameter using a range of wheel velocities.
    # For each wheel velocity, the robot scale parameter can be computed
    # by comparing the time and distance driven to the input wheel velocities.

    ##########################################
    # Feel free to change the range / step
    ##########################################
    wheel_velocities_range = range(20, 110, 15)
    delta_times = []
    delta_dists = []
    set_time = 10

    for wheel_vel in wheel_velocities_range:
        print("Driving at {} ticks/s.".format(wheel_vel))
        # Repeat the test until the correct time is found.
        while True:
            
            start_now = input("Start (input any key)?: ")
            
            ppi.set_velocity([1, 0], tick=wheel_vel, time=set_time)
            # delta_time = input("Input the time to drive in seconds: ")
            delta_dist = input("Input the distance it drove in m: ")
            try:
                # delta_time = float(delta_time)
                delta_dist = float(delta_dist)
            except ValueError:
                print("Time must be a number.")
                continue

            # Drive the robot at the given speed for the given time
            # ppi.set_velocity([1, 0], tick=wheel_vel, time=delta_time)
            
            # ppi.set_velocity([1, 0], tick=wheel_vel, time=set_time)

            delta_dists.append(delta_dist)
            print("Recording that the robot drove {:.2f} in 10 seconds at wheel speed {}.\n".format(delta_dist, wheel_vel))
            break
            
            #uInput = input("Did the robot travel 1m?[y/N]")
            #if uInput == 'y':
                # delta_times.append(delta_time)
                
                # print("Recording that the robot drove 1m in {:.2f} seconds at wheel speed {}.\n".format(delta_time, wheel_vel))
                # break

    # Once finished driving, compute the scale parameter by averaging
    num = len(wheel_velocities_range)
    scale = 0
    # for delta_time, wheel_vel in zip(delta_dists, wheel_velocities_range):
    for delta_dist, wheel_vel in zip(delta_dists, wheel_velocities_range):
        scale += delta_dist/(set_time*wheel_vel)
        
    scale = scale/num
    print("The scale parameter is estimated as {:.6f} m/ticks.".format(scale))

    return scale


def calibrateBaseline(scale):
    # Compute the robot basline parameter using a range of wheel velocities.
    # For each wheel velocity, the robot baseline parameter can be computed by
    # comparing the time elapsed and rotation completed to the input wheel
    # velocities to find out the distance between the wheels.

    ##########################################
    # Feel free to change the range / step
    ##########################################
    wheel_velocities_range = range(30, 60, 10)
    delta_times = []

    for wheel_vel in wheel_velocities_range:
        print("Driving at {} ticks/s.".format(wheel_vel))
        # Repeat the test until the correct time is found.
        while True:
            delta_time = input("Input the time to drive in seconds: ")
            try:
                delta_time = float(delta_time)
            except ValueError:
                print("Time must be a number.")
                continue
            #python3 Week02-04/calibration/wheel_calibration.py --ip 192.168.50.1 --port 8080 
            #python3 calibration/wheel_calibration.py --ip 192.168.50.1 --port 8080 

            # Spin the robot at the given speed for the given time
            ppi.set_velocity([0, 1], tick=20,turning_tick=wheel_vel, time = delta_time)

            uInput = input("Did the robot spin 360deg?[y/N]")
            if uInput == 'y':
                delta_times.append(delta_time)
                print("Recording that the robot spun 360deg in {:.2f} seconds at wheel speed {}.\n".format(delta_time,
                                                                                                           wheel_vel))
                break

    # Once finished driving, compute the basline parameter by averaging
    num = len(wheel_velocities_range)
    radius =  0
    baseline = 0
    for delta_time, wheel_vel in zip(delta_times, wheel_velocities_range):
        calibratedVel = wheel_vel*scale
        
        # radius += (calibratedVel*delta_time)/(2*np.pi)
        
        radius += (calibratedVel*delta_time)/(np.pi)
        
        
        
        
     # : replace with your code to compute the baseline parameter using scale, wheel_vel, and delta_time
     
    baseline = radius/num
    
    print("The baseline parameter is estimated as {:.6f} m.".format(baseline))

    return baseline


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", metavar='', type=str, default='localhost')
    parser.add_argument("--port", metavar='', type=int, default=40000)
    args, _ = parser.parse_known_args()

    ppi = PenguinPi(args.ip,args.port)

    # calibrate pibot scale and baseline
    dataDir = "{}/calibration/param/".format(os.getcwd())

    # # print('Calibrating PiBot scale...\n')
    scale = calibrateWheelRadius()
    # scale = 2.675598708309235001e-03
    fileNameS = "{}scale.txt".format(dataDir)
    np.savetxt(fileNameS, np.array([scale]), delimiter=',')

    print('Calibrating PiBot baseline...\n')
   # baseline = calibrateBaseline(scale)
    fileNameB = "{}baseline.txt".format(dataDir)
    np.savetxt(fileNameB, np.array([baseline]), delimiter=',')
    
    # driveMeters(1,scale)

    print('Finished calibration')
