package com.nick.wood.physics.particle_system_dynamics_verbose;

import com.nick.wood.maths.objects.vector.Vec3d;

/**Z down frame of reference
 *
 */
public class Gravity implements NaryForce {

	private static final double G = 9.81;

	@Override
	public Vec3d calculateForceOnParticle(Particle ... particle) {
		return new Vec3d(0.0, 0.0, -particle[0].getMass() * G);
	}
}