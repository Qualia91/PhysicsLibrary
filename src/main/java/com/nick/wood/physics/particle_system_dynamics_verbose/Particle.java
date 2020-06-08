package com.nick.wood.physics.particle_system_dynamics_verbose;

import com.nick.wood.maths.objects.QuaternionD;
import com.nick.wood.maths.objects.vector.Vec3d;
import com.nick.wood.maths.objects.vector.Vecd;
import com.nick.wood.physics.Body;

import java.util.ArrayList;
import java.util.UUID;

public class Particle implements Body {

	private final UUID uuid = UUID.randomUUID();
	private final double mass;
	private Vec3d position;
	private Vec3d velocity;
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

	public UUID getUuid() {
		return uuid;
	}

	@Override
	public Vecd getOrigin() {
		return position;
	}

	@Override
	public QuaternionD getRotation() {
		return QuaternionD.Rotation(Vec3d.ZERO);
	}

	public void updateState(Vec3d newPos, Vec3d newVel) {
		this.position = newPos;
		this.velocity = newVel;
	}
}
