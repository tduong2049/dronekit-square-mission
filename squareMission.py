from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
from math import radians, cos
import time

"""
Function that sets the drone to GUIDED mode
and attempts to arm it.
"""
def setupDrone(currentDrone):

    #Sets the drone to GUIDED mode
    print("\nSetting drone to GUIDED mode now...")
    currentDrone.mode = VehicleMode("GUIDED")
    print("Drone is in GUIDED mode.")

    #Arm the drone
    print("\nDrone is armable. Arming it now...")
    currentDrone.armed = True
    time.sleep(1)

    #If the drone does not successfully arm,
    #attempt to arm it again.
    while not currentDrone.armed:
        print("--Drone is still arming...")
        currentDrone.armed = True
        time.sleep(1)

    print("\nDrone is armed!")

"""
Function that checks the current drone's altitude and prints it every second.
Once its target altitude is reached, print it.
"""
def altitudeCheck(currentDrone, targetAltitude):

    print("Taking off now.")
    # While the target altitude has yet to be reached, print the drone's
    # current altitude every couple of seconds.
    while currentDrone.location.global_relative_frame.alt < (targetAltitude - 0.1):
        print "--Altitude: ", currentDrone.location.global_relative_frame.alt
        time.sleep(1)

    # Once the target altitude is reached, print it.
    print("--Target Altitude of " + str(targetAltitude) + "m reached.")

"""
Function that takes in the drone's current location and calculates a new
latitude coordinate to go to based on the given distance/directon to travel.
"""
def calculateNewLat(currentDrone, travelDistance):

    #Store the current drone's latitude in degrees and radians
    orgLatDeg = currentDrone.location.global_frame.lat
    orgLatRad = radians(orgLatDeg)

    #Formula to calculate meters per degree of latitude at current latitude
    latMetersPerDeg = 111132.92 - 559.82*cos(2 * orgLatRad) + 1.175*cos(4 * orgLatRad) - 0.0023*cos(6 * orgLatRad)

    #New latitude coordinate can be calculated by adding
    #the ratio of travelDistance to meters per degree to current latitude
    newLat = orgLatDeg + (travelDistance/latMetersPerDeg)

    return newLat

"""
Function that takes in the drone's current location and calculates a new
longitude coordinate to go to based on the given distance/direction to travel.
"""
def calculateNewLong(currentDrone, travelDistance):

    #Store the current drone's latitude and longitude in degrees,
    #and its latitude in radians too.
    orgLatDeg = currentDrone.location.global_frame.lat
    orgLongDeg = currentDrone.location.global_frame.lon
    orgLatRad = radians(orgLatDeg)

    #Formula to calculate the meters per degree of longitude at current latitude
    longMetersPerDeg = 111412.84*cos(orgLatRad) - 93.5*cos(3 * orgLatRad) + 0.118*cos(5 * orgLatRad)

    #New longitude coordinate can be calculated by adding
    #the ratio of travelDistance to meters per degree to current longitude
    newLong = orgLongDeg + (travelDistance/longMetersPerDeg)

    return newLong

"""
Function that clears the currently stored mission in the drone
and stores a new one consisting of multiple waypoints.
"""
def createMission(currentDrone):

    #Download the current missions stored in the drone and wait until
    #it finishes downloading
    currentDrone.commands.download()
    currentDrone.commands.wait_ready()

    #Clear the currently stored missions in the drone
    currentDrone.commands.clear()

    #store the drone's current latitude and longitude
    homeLat = currentDrone.location.global_frame.lat
    homeLong = currentDrone.location.global_frame.lon

    #store the coordinates of the new latitude and longitude based on
    #distance and direction to be traveled.
    metersNorth = calculateNewLat(currentDrone, 1)
    metersEast = calculateNewLong(currentDrone, 1)

    #North from home location
    cmd1 = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 5, 0, 0, 0, metersNorth, homeLong, 8)

    #Northeast from home location
    cmd2 = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 5, 0, 0, 0, metersNorth, metersEast, 8)

    #East from home location
    cmd3 = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 5, 0, 0, 0, homeLat, metersEast, 8)

    #Go back to home location
    cmd4 = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 5, 0, 0, 0, homeLat, homeLong, 8)

    currentDrone.commands.add(cmd1)
    currentDrone.commands.add(cmd2)
    currentDrone.commands.add(cmd3)
    currentDrone.commands.add(cmd4)

    currentDrone.commands.upload()

def main():
    #Have the Pi connect to the pixhawk through Telem 2
    print("Attempting to connect to drone...")
    drone = connect('/dev/ttyS2', baud = 57600, wait_ready = True)

    print "Home Location: " + str(drone.location.global_frame.lat) + ", " + str(drone.location.global_frame.lon)
    #When the Pi successfully connects to the Pixhawk,
    #run the setupDrone function.
    print("\nDrone is connected, setting up now...")
    setupDrone(drone)

    #Variable that stores the altitude (in meters)
    #at which the drone should hover at.
    targetAltitude = 2.5
    print("Target altitude of " + str(targetAltitude) + "m has been set.")

    createMission(drone)

    #Have drone takeoff at targeted height
    drone.simple_takeoff(targetAltitude)

    #Run function to check the drone's current altitude
    altitudeCheck(drone, targetAltitude)

    #drone executes stored mission when its vehicle mode is set to auto
    drone.mode = VehicleMode('AUTO')

    drone.close()

main()
