package com.nick.wood.rigid_body_dynamics.particle_system_dynamics_verbose;

import com.nick.wood.rigid_body_dynamics.graphics.math.Vec3d;

import java.util.ArrayList;

public class Particle {

	private final double mass;
	private final Vec3d position;
	private final Vec3d velocity;
	private final ArrayList<NaryForce> nonGlobalForces;

	public Particle(double mass, Vec3d position, Vec3d velocity, ArrayList<NaryForce> nonGlobalForces) {
		this.mass = mass;
		this.position = position;
		this.velocity = velocity;
		this.nonGlobalForces = nonGlobalForces;
	}

	public double getMass() {
		return mass;
	}

	public Vec3d getPosition() {
		return position;
	}

	public Vec3d getVelocity() {
		return velocity;
	}

	public ArrayList<NaryForce> getNonGlobalForces() {
		return nonGlobalForces;
	}
}
