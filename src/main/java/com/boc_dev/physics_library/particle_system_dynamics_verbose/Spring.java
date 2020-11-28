package com.boc_dev.physics_library.particle_system_dynamics_verbose;

import com.boc_dev.maths.objects.vector.Vec3d;

import java.util.ArrayList;

public class Spring implements NaryForce {

	private final float restLength;
	private final float springConstant;
	private final float dampingConstant;

	public Spring(float restLength, float springConstant, float dampingConstant) {
		this.restLength = restLength;
		this.springConstant = springConstant;
		this.dampingConstant = dampingConstant;
	}

	@Override
	public Vec3d calculateForceOnParticle(Particle targetParticle, ArrayList<Particle> particles) {

		Vec3d I = Vec3d.ZERO;
		Vec3d Idot = Vec3d.ZERO;

		for (Particle otherParticle : particles) {
			if (targetParticle != otherParticle) {
				I = I.add(otherParticle.getPosition().subtract(targetParticle.getPosition()));
				Idot = Idot.add(otherParticle.getVelocity().subtract(targetParticle.getVelocity()));
			}
		}

		double Ilength = I.length();

		return (I.scale(1/Ilength)).scale(-1 * ((springConstant * (Ilength - restLength)) + (dampingConstant * (Idot.dot(I))/Ilength)));

	}
}
