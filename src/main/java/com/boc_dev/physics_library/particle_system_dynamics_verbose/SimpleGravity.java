package com.boc_dev.physics_library.particle_system_dynamics_verbose;

import com.boc_dev.maths.objects.vector.Vec3d;

import java.util.ArrayList;

/**Z down frame of reference
 *
 */
public class SimpleGravity implements NaryForce {

	private float g = 9.81f;

	public SimpleGravity() {
	}

	public SimpleGravity(float g) {
		this.g = g;
	}

	@Override
	public Vec3d calculateForceOnParticle(Particle targetParticle, ArrayList<Particle> particles) {

		for (NaryForce force : targetParticle.getForces()) {
			if (force instanceof SimpleGravity) {
				return new Vec3d(0.0, 0.0, -targetParticle.getMass() * g);
			}
		}

		return Vec3d.ZERO;

	}
}
