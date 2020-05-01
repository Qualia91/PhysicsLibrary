package com.nick.wood.physics.particle_system_dynamics_verbose;

import com.nick.wood.maths.objects.vector.Vec3d;

public class Spring implements NaryForce {

	private final double restLength;
	private final double springConstant;
	private final double dampingConstant;

	public Spring(double restLength, double springConstant, double dampingConstant) {
		this.restLength = restLength;
		this.springConstant = springConstant;
		this.dampingConstant = dampingConstant;
	}

	@Override
	public Vec3d calculateForceOnParticle(Particle... particle) {
		Vec3d I = particle[1].getPosition().subtract(particle[0].getPosition());
		Vec3d Idot = particle[1].getVelocity().subtract(particle[0].getVelocity());

		double Ilength = I.length();

		return (I.scale(1/Ilength)).scale(-1 * ((springConstant * (Ilength - restLength)) + (dampingConstant * (Idot.dot(I))/Ilength)));

	}
}
