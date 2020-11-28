package com.boc_dev.physics_library.particle_system_dynamics_verbose;

import com.boc_dev.maths.objects.vector.Vec3d;

import java.util.ArrayList;

public class ViscousDrag implements NaryForce {

	private final float coefficientOfDrag;

	public ViscousDrag(float coefficientOfDrag) {
		this.coefficientOfDrag = coefficientOfDrag;
	}

	@Override
	public Vec3d calculateForceOnParticle(Particle targetParticle, ArrayList<Particle> particles) {
		return new Vec3d(
				-coefficientOfDrag*targetParticle.getVelocity().getX(),
				-coefficientOfDrag*targetParticle.getVelocity().getY(),
				-coefficientOfDrag*targetParticle.getVelocity().getZ());
	}
}
