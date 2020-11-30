package com.boc_dev.physics_library.particle_system_dynamics_verbose;

import com.boc_dev.maths.objects.vector.Vec3d;

import java.util.ArrayList;

/**Z down frame of reference
 *
 */
public class Gravity implements NaryForce {

	private final double G;

	public Gravity() {
		this(6.6743e-11);
	}

	public Gravity(double G) {
		this.G = G;
	}

	@Override
	public Vec3d calculateForceOnParticle(Particle targetParticle, ArrayList<Particle> particles) {

		Vec3d sum = Vec3d.ZERO;

		for (Particle particle : particles) {

			if (!targetParticle.equals(particle)) {

				// get vector towards other object and distance between squared
				Vec3d vecTowards = particle.getPosition().subtract(targetParticle.getPosition());

				double len2 = vecTowards.length2();

				if (len2 < 0.01) {
					continue;
				}

				Vec3d n = vecTowards.normalise();

				// get scale of force
				double f = G * particle.getMass() * targetParticle.getMass() / len2;

				sum = sum.add(n.scale(f));

			}

		}

		return sum;

	}
}
