package com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.forces;

import com.nick.wood.rigid_body_dynamics.maths.Vec3d;
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.RigidBody;

public class Gravity implements Force{

	private static final double G = 6.6743e-11;

	@Override
	public Vec3d act(RigidBody rigidBody, RigidBody otherRigidBody) {
		// get vector towards other object and distance between squared
		Vec3d vecTowards = otherRigidBody.getOrigin().subtract(rigidBody.getOrigin());
		double len2 = vecTowards.length2();
		Vec3d n = vecTowards.normalise();

		// get scale of force
		double f = G * rigidBody.getMass() * otherRigidBody.getMass() / len2;

		return n.scale(f);
	}
}
