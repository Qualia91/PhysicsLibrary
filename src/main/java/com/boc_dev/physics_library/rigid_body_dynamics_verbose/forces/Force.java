package com.boc_dev.physics_library.rigid_body_dynamics_verbose.forces;

import com.boc_dev.maths.objects.vector.Vec3d;
import com.boc_dev.physics_library.rigid_body_dynamics_verbose.RigidBody;

import java.util.ArrayList;

public interface Force {
	Pair<Vec3d, Vec3d> act(RigidBody rigidBody, ArrayList<RigidBody> rigidBodyList);
}
