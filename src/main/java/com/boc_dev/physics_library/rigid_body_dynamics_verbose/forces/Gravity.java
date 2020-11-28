package com.boc_dev.physics_library.rigid_body_dynamics_verbose.forces;

import com.boc_dev.maths.objects.vector.Vec3d;
import com.boc_dev.maths.objects.vector.Vec3f;
import com.boc_dev.physics_library.rigid_body_dynamics_verbose.RigidBody;

import java.util.ArrayList;

public class Gravity implements Force {

	private final double G;

	public Gravity() {
		this(6.6743e-11);
	}

	public Gravity(double G) {
		this.G = G;
	}

	@Override
	public Pair<Vec3d, Vec3d> act(RigidBody rigidBody, ArrayList<RigidBody> rigidBodyList) {

		Vec3d sum = Vec3d.ZERO;

		for (RigidBody otherRigidBody : rigidBodyList) {

			if (!rigidBody.equals(otherRigidBody)) {

				// get vector towards other object and distance between squared
				Vec3d vecTowards = otherRigidBody.getOrigin().subtract(rigidBody.getOrigin());

				double len2 = vecTowards.length2();

				if (len2 < 0.01) {
					continue;
				}

				Vec3d n = vecTowards.normalise();

				// get scale of force
				double f = G * rigidBody.getMass() * otherRigidBody.getMass() / len2;

				sum = sum.add(n.scale(f));

			}

		}

		return new Pair(sum, Vec3d.ZERO);
	}

}
