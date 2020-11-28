package com.boc_dev.physics_library.particle_system_dynamics_verbose;

import com.boc_dev.maths.objects.vector.Vec3d;

import java.util.ArrayList;
import java.util.UUID;

public class Particle {

	private final UUID uuid;
	private final float mass;
	private Vec3d position;
	private Vec3d velocity;
	private final ArrayList<NaryForce> forces;

	public Particle(UUID uuid, float mass, Vec3d position, Vec3d velocity, ArrayList<NaryForce> forces) {
		this.uuid = uuid;
		this.mass = mass;
		this.position = position;
		this.velocity = velocity;
		this.forces = forces;
	}

	public float getMass() {
		return mass;
	}

	public Vec3d getPosition() {
		return position;
	}

	public Vec3d getVelocity() {
		return velocity;
	}

	public ArrayList<NaryForce> getForces() {
		return forces;
	}

	public UUID getUuid() {
		return uuid;
	}

	public void updateState(Vec3d newPos, Vec3d newVel) {
		this.position = newPos;
		this.velocity = newVel;
	}
}
