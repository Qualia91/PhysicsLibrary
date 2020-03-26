package com.nick.wood.rigid_body_dynamics.particle_system_dynamics_verbose;

import com.nick.wood.rigid_body_dynamics.graphics.math.Vec3d;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.UUID;

public class Simulation {

	private final HashMap<UUID, Particle> particles;
	private final ArrayList<NaryForce> forces;

	public Simulation(HashMap<UUID, Particle> particles, ArrayList<NaryForce> forces) {
		this.particles = particles;
		this.forces = forces;
	}

	public void eulerStep(double timeStep) {

		particles.forEach((uuid, particle) -> {
			Vec3d force = calculateForcesActingOn(particle);
			update(particle, timeStep, force);
		});

	}

	private Vec3d calculateForcesActingOn(Particle particle) {

		double xSum = 0;
		double ySum = 0;
		double zSum = 0;

		for (NaryForce force : forces) {
			Vec3d vec3d = force.calculateForceOnParticle(particle);
			xSum += vec3d.getX();
			ySum += vec3d.getY();
			zSum += vec3d.getZ();
		}

		return new Vec3d(xSum, ySum, zSum);

	}

	private void update(Particle particle, double timeStep, Vec3d force) {

		// find new velocity: old vel + force/m * timestep
		Vec3d newVel = particle.getVelocity().add(force.scale(1 / particle.getMass()).scale(timeStep));
		Vec3d newPos = particle.getPosition().add(particle.getVelocity().scale(timeStep));

		particle.updateState(newPos, newVel);

	}


	public HashMap<UUID, Particle> getParticles() {
		return particles;
	}
}
