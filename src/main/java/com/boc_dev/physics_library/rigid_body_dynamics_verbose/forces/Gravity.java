package com.boc_dev.physics_library.rigid_body_dynamics_verbose.forces;

import com.boc_dev.maths.objects.vector.Vec3d;
import com.boc_dev.physics_library.rigid_body_dynamics_verbose.RigidBody;

import java.util.ArrayList;

public class Gravity implements Force{

	private static final double G = 6.6743e-11;
	private final ArrayList<RigidBody> rigidBodyList;

	public Gravity(ArrayList<RigidBody> rigidBodyList) {
		this.rigidBodyList = rigidBodyList;
	}

	@Override
	public Vec3d actLinear(RigidBody rigidBody) {

		Vec3d sum = Vec3d.ZERO;

		for (RigidBody otherRigidBody : rigidBodyList) {

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

		return sum;
	}

	@Override
	public Vec3d actAngular(RigidBody rigidBody) {
		return Vec3d.ZERO;
	}
}
