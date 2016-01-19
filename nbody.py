#!/usr/bin/env python3

import math
from turtle import *
bgcolor("black")

# The gravitational constant G
G = 6.67428e-11

# Assumed scale: 100 pixels = 1AU.
AU = (149.6e6 * 1000)     # 149.6 million km, in meters.
SCALE = 250 / AU

class Body(Turtle):
    """Subclass of Turtle representing a gravitationally-acting body.

    Extra attributes:
    mass : mass in kg
    vx, vy: x, y velocities in m/s
    px, py: x, y positions in m
    """
    
    name = 'Body'
    mass = None
    vx = vy = 0.0
    px = py = 0.0
    
    def attraction(self, other):
        """(Body): (fx, fy)

        Returns the force exerted upon this body by the other body.
        """
        # Report an error if the other object is the same as this one.
        if self is other:
            raise ValueError("Attraction of object %r to itself requested"
                             % self.name)

        # Compute the distance of the other body. Assume same planetary plane, 2-D problem
        sx, sy = self.px, self.py
        ox, oy = other.px, other.py
        dx = (ox-sx)
        dy = (oy-sy)
        d = math.sqrt(dx**2 + dy**2)

        # Report an error if the distance is zero; otherwise we'll
        # get a ZeroDivisionError exception further down.
        if d == 0:
            raise ValueError("Collision between objects %r and %r"
                             % (self.name, other.name))

        # Compute the force of attraction
        f = G * self.mass * other.mass / (d**2)

        # Compute the direction of the force.
        theta = math.atan2(dy, dx)
        fx = math.cos(theta) * f
        fy = math.sin(theta) * f
        return fx, fy

def update_info(step, bodies):
    """(int, [Body])
    
    Displays information about the status of the simulation.
    """
    print('Step #{}'.format(step))
    for body in bodies:
        s = '{:<8}  Pos.={:>6.2f} {:>6.2f} [ {:>6.4f} ] Vel.={:>10.3f} {:>10.3f}'.format(
            body.name, body.px/AU, body.py/AU, math.sqrt((body.px/AU)**2 + (body.py/AU)**2), body.vx, body.vy)
        print(s)
    print()
    
def loop(bodies):
    """([Body])

    Never returns; loops through the simulation, updating the
    positions of all the provided bodies.
    """
    timestep = 24*3600  # One day
    
    for body in bodies:
        body.penup()
        body.hideturtle()

    step = 1
    while True:
        update_info(step, bodies)
        step += 1

        force = {}
        for body in bodies:
            # Add up all of the forces exerted on 'body'.
            total_fx = total_fy = 0.0
            for other in bodies:
                # Don't calculate the body's attraction to itself
                if body is other:
                    continue
                fx, fy = body.attraction(other)
                total_fx += fx
                total_fy += fy

            # Record the total force exerted.
            force[body] = (total_fx, total_fy)

        # Update velocities based upon on the force.
        for body in bodies:
            fx, fy = force[body]
            body.vx += fx / body.mass * timestep
            body.vy += fy / body.mass * timestep

            # Update positions
            body.px += body.vx * timestep
            body.py += body.vy * timestep
            body.goto(body.px*SCALE, body.py*SCALE)
            body.dot(3)


def main():
    sun = Body()
    sun.name = 'Sun'
    sun.mass = 1.98892 * 10**30
    sun.pencolor('yellow')

    earth = Body()
    earth.name = 'Earth'
    earth.mass = 5.9742 * 10**24
    earth.px = -0.70710678118*AU
    earth.py = 0.70710678118*AU
    earth.vy = 29.783 * 1000 * 0.70710678118           # 29.783 km/sec
    earth.vx = 29.783 * 1000 * 0.70710678118
    earth.pencolor('blue')

    uranus = Body()
    uranus.name = 'Uranus'
    uranus.mass = 8.681 * 10**25
    uranus.px = -1*AU
    uranus.vy = 29.783 * 1000            # 29.783 km/sec
    uranus.pencolor('violet')

    neptune = Body()
    neptune.name = 'Neptune'
    neptune.mass = 1.0243 * 10**26
    neptune.px = 1*AU
    neptune.vy = -29.783 * 1000            # 29.783 km/sec
    neptune.pencolor('brown')
    
    mars = Body()
    mars.name = 'Mars'
    mars.mass = 6.4171 * 10**23
    mars.px = 0.70710678118*AU
    mars.py = -0.70710678118*AU
    mars.vy = -29.783 * 1000 * 0.70710678118           # 29.783 km/sec
    mars.vx = -29.783 * 1000 * 0.70710678118
    mars.pencolor('red')   

    venus = Body()
    venus.name = 'Venus'
    venus.mass = 4.8675 * 10**24
    venus.px = 0.70710678118*AU
    venus.py = 0.70710678118*AU
    venus.vy = -29.783 * 1000 * 0.70710678118           # 29.783 km/sec
    venus.vx = 29.783 * 1000 * 0.70710678118
    venus.pencolor('yellow')   
    
    mercury = Body()
    mercury.name = 'Mercury'
    mercury.mass = 3.3011 * 10**23
    mercury.px = -0.70710678118*AU
    mercury.py = -0.70710678118*AU
    mercury.vy = 29.783 * 1000 * 0.70710678118           # 29.783 km/sec
    mercury.vx = -29.783 * 1000 * 0.70710678118
    mercury.pencolor('white')   

    # Jupiter placed in Earth's orbital plane
    jupiter = Body()
    jupiter.name = 'Jupiter'
    jupiter.mass = 1.8986 * 10**27
    jupiter.py = -1*AU
    jupiter.vx = -29.783 * 1000            # 29.783 km/sec
    jupiter.pencolor('orange')
    
    # Saturn placed in Earth's orbital plane
    saturn = Body()
    saturn.name = 'Saturn'
    saturn.mass = 5.6836 * 10**26
    saturn.py = 1*AU
    saturn.vx = 29.783 * 1000            # 29.783 km/sec
    saturn.pencolor('green')

    loop([sun, earth, uranus, neptune, mars, venus, mercury, jupiter, saturn])

if __name__ == '__main__':
    main()
