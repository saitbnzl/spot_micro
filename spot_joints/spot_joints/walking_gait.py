import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import math
from kinematics import *
class Leg:
    def __init__(self, index, origin):
        self.origin = origin
        self.index = index

    def calculate_leg_positions(self, phase):
        radius = 5.0

        x = self.origin[0] - math.cos(phase * 360 * math.pi / 180) * radius
        y = self.origin[1] + math.sin(phase * 360 * math.pi / 180) * radius
        y = max(-radius * 0.9, y - self.origin[1])
        z = self.origin[2]

        self.position = [x, y, z]

        return x, y, z

def generate_leg_positions(legs, phase):
    leg_positions = []
    for leg in legs:
        x, y, z = leg.calculate_leg_positions(phase if leg.index % 2 == 0 else phase + 0.5)
        leg_positions.append([x, y, z])
        #print(x,y,z)
        print(calculate_joint_angles(x,y,z))
    return leg_positions

def calculate_joint_angles(x, y, z):
    leg_angles = legIK([x, y, z])
    return leg_angles

def visualize_leg_positions(ax, leg_positions, frame):
    ax.cla()

    for i, pos in enumerate(leg_positions):
        ax.scatter(pos[0], pos[1], pos[2], label=f'Leg {i}', color='r' if i % 2 == 0 else 'g')

    ax.set_title(f'Frame {frame + 1}/{num_frames}')
    ax.set_xlim(-150, 150)
    ax.set_ylim(-150, 150)
    ax.set_zlim(-150, 150)
    ax.view_init(elev=-60, azim=90)
    plt.pause(.01)

def animate_quadruped(num_frames):
    legs = [Leg(0, [100,-100,100]), Leg(1, [100,-100,-100, 0]), 
            Leg(2, [-100,-100,100]), Leg(3, [-100,-100,-100])]
    for frame in range(num_frames):
        phase = frame / num_frames
        leg_positions = generate_leg_positions(legs, phase)
        visualize_leg_positions(ax, leg_positions, frame)

# Set the number of frames, legs, and leg distance
num_frames = 30

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')
ax.legend()

# Call the animation function
while True:
    animate_quadruped(num_frames)

# Display the final plot
plt.show()
