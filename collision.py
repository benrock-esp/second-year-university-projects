import matplotlib.pyplot as plt
import matplotlib.animation as animation
import itertools
import numpy as np
from PIL import Image


class Canvas:
    def __init__(self):
        self.size = 20
        self.particles = []
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1, 1, 1)

    def add_particle(self, particle):
        self.particles.append(particle)

    def update_particles(self):
        self.ax.clear()
        for particle in self.particles:
            particle.move()
            particle.draw()

    def fix_axes(self):
        self.ax.set_xlim((-self.size / 2, self.size / 2))
        self.ax.set_ylim((-self.size / 2, self.size / 2))
        self.ax.set_aspect("equal")

    def check_collisions(self):
        combinations = itertools.combinations(range(len(self.particles)), 2)
        for i, j in combinations:
            self.particles[i].collide(self.particles[j])
        for particle in self.particles:
            particle.wall_collision()


class Particle:
    def __init__(self, canvas, mass, position, velocity):
        self.canvas = canvas
        self.mass = mass
        self.position = np.array(position, dtype=float)
        self.velocity = np.array(velocity, dtype=float)
        self.canvas.add_particle(self)
        self.color = "black"

    def move(self):
        self.position += self.velocity

    def draw(self):
        self.canvas.ax.plot(self.position[0], self.position[1], "o", color=self.color)

    def collide(self, other):
        # Vector from this particle to the other
        displacement = other.position - self.position
        distance = np.linalg.norm(displacement)

        # Check for collision (particles overlap within a small threshold)
        if distance < 0.1:  # Adjust threshold for larger particles
            # Normalize the displacement vector
            normal = displacement / distance

            # Relative velocity
            relative_velocity = self.velocity - other.velocity

            # Velocity component along the normal
            velocity_along_normal = np.dot(relative_velocity, normal)

            if velocity_along_normal > 0:
                return  # No collision if they're moving apart

            # Compute impulse scalar using conservation of momentum
            impulse = (-(1 + 1) * velocity_along_normal) / (1 / self.mass + 1 / other.mass)

            # Apply impulse to both particles
            impulse_vector = impulse * normal
            self.velocity -= impulse_vector / self.mass
            other.velocity += impulse_vector / other.mass

    def wall_collision(self):
        # Check collision with vertical walls
        if self.position[0] <= -self.canvas.size / 2 or self.position[0] >= self.canvas.size / 2:
            self.velocity[0] = -self.velocity[0]  # Reverse x-velocity

        # Check collision with horizontal walls
        if self.position[1] <= -self.canvas.size / 2 or self.position[1] >= self.canvas.size / 2:
            self.velocity[1] = -self.velocity[1]  # Reverse y-velocity

    def apply_velocity_decay(self, decay_factor_per_second, fps):
        # Calculate decay per frame
        decay_per_frame = decay_factor_per_second ** (1 / fps)
        self.velocity *= decay_per_frame


# Create canvas and randomly initialized particles
canvas = Canvas()
num_particles = 20  # Total number of particles
np.random.seed(42)  # For reproducibility

for _ in range(num_particles):
    position = np.random.uniform(-canvas.size / 2, canvas.size / 2, size=2)
    velocity = np.random.uniform(-0.3, 0.3, size=2)
    mass = 1  # Assuming all particles have the same mass
    Particle(canvas, mass, position, velocity)

# Simulation parameters
fps = 30
decay_per_second = 0.99  # Slower decay for more gradual reduction


def animate(i):
    for particle in canvas.particles:
        particle.apply_velocity_decay(decay_per_second, fps)
    canvas.update_particles()
    canvas.check_collisions()
    canvas.fix_axes()
    return canvas.ax.get_figure(),


# Save animation as GIF using PillowWriter
writer = animation.PillowWriter(fps=fps)
anim = animation.FuncAnimation(canvas.fig, animate, frames=450, interval=1000 / fps, blit=False)
anim.save("particles_2d_collisions_slow_decay.gif", writer=writer)
