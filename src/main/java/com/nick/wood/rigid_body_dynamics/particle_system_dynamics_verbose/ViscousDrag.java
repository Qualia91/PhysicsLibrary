package com.nick.wood.rigid_body_dynamics.particle_system_dynamics_verbose;

import com.nick.wood.rigid_body_dynamics.graphics.math.Vec3d;

public class ViscousDrag implements NaryForce {

	private final double coefficientOfDrag;

	public ViscousDrag(double coefficientOfDrag) {
		this.coefficientOfDrag = coefficientOfDrag;
	}

	@Override
	public Vec3d calculateForceOnParticle(Particle ... particle) {
		return new Vec3d(
				-coefficientOfDrag*particle[0].getVelocity().getX(),
				-coefficientOfDrag*particle[0].getVelocity().getY(),
				-coefficientOfDrag*particle[0].getVelocity().getZ());
	}
}
