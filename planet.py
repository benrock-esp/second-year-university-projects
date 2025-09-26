import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from mpl_toolkits.mplot3d import Axes3D

G = 6.67 * 10**-11

# Solar system class definition
class SolarSystem:
    def __init__(self, ax, mass, initial_position=(0, 0, 0), initial_velocity=(0, 0, 0)):
        self.mass = mass  
        self.position = np.array(initial_position, dtype=float)  # Center of the solar system
        self.velocity = np.array(initial_velocity, dtype=float)  # Velocity of the solar system
        self.planets = [] 
        self.meteors = []
        self.ax = ax 
        self.dT = 1 

    def add_planet(self, planet):
        self.planets.append(planet)

    def add_meteor(self, meteor):
        self.meteors.append(meteor)

    def update_planets(self):
        for planet in self.planets:
            planet.update(self.position, self.meteors)  # Update position with meteors affecting them
            planet.draw()  # Draw the planet and its trail

    def update_meteors(self):
        for meteor in self.meteors:
            meteor.update(self.planets)  # Meteors affected by planets
            meteor.draw()  # Draw the meteor

    def check_collisions(self):
        for i, planet1 in enumerate(self.planets):
            for planet2 in self.planets[i + 1:]:
                if planet1.check_collision(planet2):  # Check for collisions
                    planet1.handle_collision(planet2)  # Handle merging
                    self.planets.remove(planet2)  # Remove the smaller planet

    def fix_axes(self):
        self.ax.set_xlim(-2000, 2000)
        self.ax.set_ylim(-2000, 2000)
        self.ax.set_zlim(-2000, 2000)

# Planet class
class Planet:
    def __init__(self, solar_system, mass, position, velocity):
        self.solar_system = solar_system
        self.mass = mass
        self.position = np.array(position, dtype=float) + solar_system.position
        self.velocity = np.array(velocity, dtype=float) + solar_system.velocity
        self.solar_system.add_planet(self)
        self.color = "blue"
        self.trail = []

    def update(self, solar_system_position, meteors):
        # Force from the central sun
        distance_vector = solar_system_position - self.position
        distance = np.linalg.norm(distance_vector)
        if distance > 0:
            force_direction = distance_vector / distance
            force_magnitude = G * self.solar_system.mass * self.mass / (distance ** 2)
            force = force_magnitude * force_direction
        else:
            force = np.zeros(3)

        # Force from meteors
        for meteor in meteors:
            distance_vector = meteor.position - self.position
            distance = np.linalg.norm(distance_vector)
            if distance > 0:
                force_direction = distance_vector / distance
                force_magnitude = G * meteor.mass * self.mass / (distance ** 2)
                force += force_magnitude * force_direction

        # Update velocity and position
        self.velocity += (force / self.mass) * self.solar_system.dT
        self.position += self.velocity * self.solar_system.dT
        self.trail.append(self.position.copy())

    def draw(self):
        self.solar_system.ax.scatter(*self.position, color=self.color, s=50)
        if len(self.trail) > 1:
            trail = np.array(self.trail[-50:])  # Limit trail length
            self.solar_system.ax.plot(trail[:, 0], trail[:, 1], trail[:, 2], color=self.color, linewidth=2)

    def check_collision(self, other):
        distance = np.linalg.norm(self.position - other.position)
        return distance < 100  # Collision threshold

    def handle_collision(self, other):
        total_mass = self.mass + other.mass
        new_velocity = (self.velocity * self.mass + other.velocity * other.mass) / total_mass
        self.mass = total_mass
        self.velocity = new_velocity
        self.color = "purple"

# Meteor class
class Meteor:
    def __init__(self, solar_system, mass, position, velocity):
        self.solar_system = solar_system
        self.mass = mass
        self.position = np.array(position, dtype=float)
        self.velocity = np.array(velocity, dtype=float)
        self.solar_system.add_meteor(self)
        self.color = "red"

    def update(self, planets):
        force = np.zeros(3)
        for planet in planets:
            distance_vector = planet.position - self.position
            distance = np.linalg.norm(distance_vector)
            if distance > 0:
                force_direction = distance_vector / distance
                force_magnitude = G * planet.mass * self.mass / (distance ** 2)
                force += force_magnitude * force_direction

        # Update velocity and position
        self.velocity += (force / self.mass) * self.solar_system.dT
        self.position += self.velocity * self.solar_system.dT

    def draw(self):
        self.solar_system.ax.scatter(*self.position, color=self.color, s=10)

# Random meteor generator
def generate_random_meteor(solar_system):
    position = np.random.uniform(-12000, 12000, size=3)
    velocity = np.random.uniform(-50, 50, size=3)
    mass = np.random.uniform(1e2, 1e3)  # Random mass
    return Meteor(solar_system, mass, position, velocity)

# Set up the 3D plot
fig = plt.figure(figsize=(8, 8))
ax = fig.add_subplot(111, projection='3d')

# Initialize solar system
SolarSys = SolarSystem(ax, mass=1e7)

# Add the sun
sun = Planet(SolarSys, mass=1e7, position=(0, 0, 0), velocity=(0, 0, 0))
sun.color = "yellow"

# Add planets
planet1 = Planet(SolarSys, mass=1e3, position=(2000, 0, 0), velocity=(0, 80, 20))
planet2 = Planet(SolarSys, mass=5e2, position=(0, -3000, 1000), velocity=(100, 0, -30))

# Fix axes
SolarSys.fix_axes()

# Animation update function
def update(frame):
    if frame % 20 == 0:  # Add a meteor every 20 frames
        generate_random_meteor(SolarSys)

    ax.cla()
    SolarSys.fix_axes()
    SolarSys.update_planets()
    SolarSys.update_meteors()
    SolarSys.check_collisions()

# Create animation
frames = 500
animation = FuncAnimation(fig, update, frames=frames, interval=50, repeat=False)

# Save animation
writer = PillowWriter(fps=20)
animation.save("enhanced_solar_system.gif", writer=writer)

print("Animation saved as solar_system.gif")
