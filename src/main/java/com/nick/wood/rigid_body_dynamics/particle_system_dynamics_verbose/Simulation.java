package com.nick.wood.rigid_body_dynamics.particle_system_dynamics_verbose;

import com.nick.wood.rigid_body_dynamics.graphics.math.Vec3d;

import java.util.ArrayList;

public class Simulation {

	private final ArrayList<Particle> particles;
	private final ArrayList<NaryForce> forces;

	public Simulation(ArrayList<Particle> particles, ArrayList<NaryForce> forces) {
		this.particles = particles;
		this.forces = forces;
	}

	public void eulerStep(double timeStep) {

		for (Particle particle : particles) {

			Vec3d force = calculateForcesActingOn(particle);

			Particle newParticle = nextTimestep(particle, timeStep, force);

		}

	}

	private Vec3d calculateForcesActingOn(Particle particle) {
		return Vec3d.ZERO;
	}

	private Particle nextTimestep(Particle particle, double timeStep, Vec3d force) {

		// find new velocity: old vel + force/m * timestep
		Vec3d newVel = particle.getVelocity().add(force.scale(1 / particle.getMass()).scale(timeStep));
		Vec3d newPos = particle.getPosition().add(particle.getVelocity().scale(timeStep));

		return new Particle(particle.getMass(), newPos, newVel, new ArrayList<>());

	}


}
