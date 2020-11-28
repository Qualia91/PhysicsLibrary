package com.boc_dev.physics_library.particle_system_dynamics_verbose;

import com.boc_dev.maths.objects.vector.Vec3d;

import java.util.ArrayList;
import java.util.Optional;

public class ParticleSimulation {

	private static final float collisionConstant = 0.6f;

	public void iterate(double timeStep, ArrayList<Particle> particles, ArrayList<Plane> planes) {

		for (Particle particle : particles) {
			Vec3d force = calculateForcesActingOn(particle, particles);
			update(particle, timeStep, force, planes, collisionConstant);
		}
	}

	private Vec3d calculateForcesActingOn(Particle particle, ArrayList<Particle> particles) {

		double xSum = 0;
		double ySum = 0;
		double zSum = 0;

		for (NaryForce force : particle.getForces()) {
			Vec3d vec3d = force.calculateForceOnParticle(particle, particles);
			xSum += vec3d.getX();
			ySum += vec3d.getY();
			zSum += vec3d.getZ();
		}

		return new Vec3d(xSum, ySum, zSum);

	}

	private void update(Particle particle, double timeStep, Vec3d force, ArrayList<Plane> planes, float collisionConstant) {

		// find new velocity: old vel + force/m * timestep
		Vec3d newVel = particle.getVelocity().add(force.scale(1 / particle.getMass()).scale(timeStep));
		Vec3d newPos = particle.getPosition().add(newVel.scale(timeStep));
		Optional<Vec3d> vec3d = collisionDetection(newPos, newVel, planes, collisionConstant);
		if (vec3d.isEmpty()) {
			particle.updateState(newPos, newVel);
		} else {
			particle.updateState(newPos, vec3d.get());
		}

	}

	private Optional<Vec3d> collisionDetection(Vec3d newPos, Vec3d velocity, ArrayList<Plane> planes, float collisionConstant) {

		double sumX = 0;
		double sumY = 0;
		double sumZ = 0;

		boolean collides = false;

		for (Plane plane : planes) {
			// if this is bigger than 0 it hasn't collided
			if (newPos.subtract(plane.getCenter()).dot(plane.getNormal()) <= 0) {
				// calculate normal component of vector
				Vec3d normalVecCom = plane.getNormal().scale(plane.getNormal().dot(velocity)).scale(-collisionConstant);
				sumX += normalVecCom.getX();
				sumY += normalVecCom.getY();
				sumZ += normalVecCom.getZ();
				collides = true;
			}
		}

		if (collides) {
			return Optional.of(new Vec3d(sumX, sumY, sumZ));
		} else {
			return Optional.empty();
		}
	}

}
