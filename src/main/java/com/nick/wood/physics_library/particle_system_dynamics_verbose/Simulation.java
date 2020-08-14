package com.nick.wood.physics_library.particle_system_dynamics_verbose;

import com.nick.wood.physics_library.Body;
import com.nick.wood.physics_library.SimulationInterface;
import com.nick.wood.maths.objects.vector.Vec3d;

import java.util.ArrayList;
import java.util.Optional;

public class Simulation {

	private final ArrayList<Body> particles;
	private final ArrayList<NaryForce> forces;
	private final ArrayList<Plane> planes;
	private static final double COLLISION_CONST = 0.6;

	public Simulation(ArrayList<Body> particles, ArrayList<NaryForce> forces, ArrayList<Plane> planes) {
		this.particles = particles;
		this.forces = forces;
		this.planes = planes;
	}

	public void eulerStep(double timeStep) {

		for (Body particle : particles) {
			Vec3d force = calculateForcesActingOn((Particle) particle);
			update((Particle) particle, timeStep, force);
		}
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

		for (NaryForce force : particle.getNonGlobalForces()) {
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
		Optional<Vec3d> vec3d = collisionDetection(newPos, newVel);
		if (vec3d.isEmpty()) {
			particle.updateState(newPos, newVel);
		} else {
			particle.updateState(newPos, vec3d.get());
		}

	}

	private Optional<Vec3d> collisionDetection(Vec3d newPos, Vec3d velocity) {

		double sumX = 0;
		double sumY = 0;
		double sumZ = 0;

		boolean collides = false;

		for (Plane plane : planes) {
			// if this is bigger than 0 it hasnt collided
			if (newPos.subtract(plane.getCenter().add(new Vec3d(0.5, 0.5, 0.5))).dot(plane.getNormal()) <= 0) {
				// calculate normal component of vector
				Vec3d normalVecCom = plane.getNormal().scale(plane.getNormal().dot(velocity)).scale(-COLLISION_CONST);
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

	public void iterate(double deltaSeconds) {

	}

	public ArrayList<Body> getBodies() {
		return particles;
	}

	public ArrayList<Plane> getPlanes() {
		return planes;
	}

}
