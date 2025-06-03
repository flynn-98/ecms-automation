from robot import Robot
from RobotCalibration import RobotCalibration

# Create a Robot instance
robot = Robot()

# Create a RobotCalibration instance
calib = RobotCalibration(robot)

# Collect exactly 15 data points (User must input x, y coordinates)
for i in range(15):
    x = float(input(f"Enter world X coordinate for point {i+1}: "))
    y = float(input(f"Enter world Y coordinate for point {i+1}: "))
    calib.collect_data(x, y)
    calib.save_data()  # Save data after each collection

# Perform calibration once all 15 data points are collected
R, T = calib.calibrate()
