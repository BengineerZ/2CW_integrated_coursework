#!/usr/bin/python3
from Box2D.examples.framework import (Framework, Keys, main)
from Box2D import (b2EdgeShape, b2FixtureDef, b2PolygonShape)
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import os, time, sys
from multiprocessing import Process

num_of_floors = 3
floor_width = 20 # cm
floor_thickness = 0.5 # cm
floor_height = 20 # cm

second_building_offset = 40 # cm

floor_mass = 1.487 # kg
spring_k = 2099 # N/m

pendulum_mass = 0.5 # kg
pendulum_length = 6.895 # cm
pendulum_size = 2 # cm
pendulum_damping = 2 * 1.819 # Ns/cm

class Simulation (Framework):
	name = "Simulation"
	max_depth = 4

	def __init__(self):
		Framework.__init__(self)

		self.world.gravity = 100 * self.world.gravity # convert from m/s^2 to cm/s^2
		self.time = 0

		self.ground = self.world.CreateStaticBody(
			position=(0, 0),
			shapes=[b2EdgeShape(vertices=[(-200, 0), (200, 0)])])

		self.pendulum = self.world.CreateDynamicBody(
			position=(0, num_of_floors * floor_height - pendulum_length),
			shapes=[b2PolygonShape(box=(pendulum_size, pendulum_size))],
			linearDamping=0)
		self.pendulum.mass = pendulum_mass

		self.base = self.world.CreateStaticBody(
			position=(0, floor_thickness),
			shapes=[b2PolygonShape(box=(floor_width / 2, floor_thickness))])

		self.floor = []
		for i in range(num_of_floors):
			self.floor.append(self.world.CreateDynamicBody(
				position=(0, floor_thickness + (i + 1) * floor_height),
				shapes=[b2PolygonShape(box=(floor_width / 2, floor_thickness))]))
			self.floor[i].mass = floor_mass

		self.base2 = self.world.CreateStaticBody(
			position=(second_building_offset, floor_thickness),
			shapes=[b2PolygonShape(box=(floor_width / 2, floor_thickness))])

		self.floor2 = []
		for i in range(num_of_floors):
			self.floor2.append(self.world.CreateDynamicBody(
				position=(second_building_offset, floor_thickness + (i + 1) * floor_height),
				shapes=[b2PolygonShape(box=(floor_width / 2, floor_thickness))]))
			self.floor2[i].mass = floor_mass

		# Building 1 joints
		floor_edge_vec = (floor_width / 2, 0)
		self.world.CreateDistanceJoint(
			bodyA=self.base, bodyB=self.floor[0],
			anchorA=self.base.position + floor_edge_vec, anchorB=self.floor[0].position + floor_edge_vec,
			dampingRatio=1)
		self.world.CreateDistanceJoint(
			bodyA=self.base, bodyB=self.floor[0],
			anchorA=self.base.position - floor_edge_vec, anchorB=self.floor[0].position - floor_edge_vec,
			dampingRatio=1)

		for i in range(num_of_floors - 1):
			self.world.CreateDistanceJoint(
				bodyA=self.floor[i], bodyB=self.floor[i+1],
				anchorA=self.floor[i].position + floor_edge_vec, anchorB=self.floor[i+1].position + floor_edge_vec,
				dampingRatio=1)
			self.world.CreateDistanceJoint(
				bodyA=self.floor[i], bodyB=self.floor[i+1],
				anchorA=self.floor[i].position - floor_edge_vec, anchorB=self.floor[i+1].position - floor_edge_vec,
				dampingRatio=1)

		# Building 2 joints
		floor_edge_vec = (floor_width / 2, 0)
		self.world.CreateDistanceJoint(
			bodyA=self.base2, bodyB=self.floor2[0],
			anchorA=self.base2.position + floor_edge_vec, anchorB=self.floor2[0].position + floor_edge_vec,
			dampingRatio=1)
		self.world.CreateDistanceJoint(
			bodyA=self.base2, bodyB=self.floor2[0],
			anchorA=self.base2.position - floor_edge_vec, anchorB=self.floor2[0].position - floor_edge_vec,
			dampingRatio=1)

		for i in range(num_of_floors - 1):
			self.world.CreateDistanceJoint(
				bodyA=self.floor2[i], bodyB=self.floor2[i+1],
				anchorA=self.floor2[i].position + floor_edge_vec, anchorB=self.floor2[i+1].position + floor_edge_vec,
				dampingRatio=1)
			self.world.CreateDistanceJoint(
				bodyA=self.floor2[i], bodyB=self.floor2[i+1],
				anchorA=self.floor2[i].position - floor_edge_vec, anchorB=self.floor2[i+1].position - floor_edge_vec,
				dampingRatio=1)

		# Pendulum joint
		self.world.CreateDistanceJoint(
			bodyA=self.floor[-1], bodyB=self.pendulum,
			anchorA=self.floor[-1].position, anchorB=self.pendulum.position)

	def Step(self, settings):
		Framework.Step(self, settings)

		# Apply external force

		# Apply spring force to building 1
		x0 = 0 - self.floor[0].position.x
		x1 = self.floor[1].position.x - self.floor[0].position.x
		self.floor[0].ApplyForce((spring_k * (x0 + x1), 0), (0, 0), True)

		xf = self.floor[-2].position.x - self.floor[-1].position.x
		self.floor[-1].ApplyForce((spring_k * xf, 0), (0, 0), True)
	
		for i in range(1, num_of_floors - 1):
			x_bot = self.floor[i-1].position.x - self.floor[i].position.x
			x_top = self.floor[i+1].position.x - self.floor[i].position.x
			self.floor[i].ApplyForce((spring_k * (x_bot + x_top), 0), (0, 0), True)

		# Apply pendulum damping
		rel_vel = self.pendulum.linearVelocity.x - self.floor[-1].linearVelocity.x
		self.pendulum.ApplyForce((-pendulum_damping * rel_vel, 0), (0, 0), True)

		# Apply spring force to building 2
		x0 = self.base2.position.x - self.floor2[0].position.x
		x1 = self.floor2[1].position.x - self.floor2[0].position.x
		self.floor2[0].ApplyForce((spring_k * (x0 + x1), 0), (0, 0), True)

		xf = self.floor2[-2].position.x - self.floor2[-1].position.x
		self.floor2[-1].ApplyForce((spring_k * xf, 0), (0, 0), True)
	
		for i in range(1, num_of_floors - 1):
			x_bot = self.floor2[i-1].position.x - self.floor2[i].position.x
			x_top = self.floor2[i+1].position.x - self.floor2[i].position.x
			self.floor2[i].ApplyForce((spring_k * (x_bot + x_top), 0), (0, 0), True)

		# Logging
		print(str(self.time) + " " + str(self.floor[-1].position.x), file=open("last_data_point.dat", "w"), flush=True)
		self.time += 1 / 60
	
	def Keyboard(self, key):
		if not self.floor[-1]:
			return

		if key == Keys.K_w:
			self.floor[-1].ApplyLinearImpulse((-100, 0), (0, 0), True)
			self.floor2[-1].ApplyLinearImpulse((-100, 0), (0, 0), True)

if __name__ == "__main__":
	main(Simulation)

