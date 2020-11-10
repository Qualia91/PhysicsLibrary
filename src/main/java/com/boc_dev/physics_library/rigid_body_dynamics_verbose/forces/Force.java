package com.boc_dev.physics_library.rigid_body_dynamics_verbose.forces;

import com.boc_dev.maths.objects.vector.Vec3d;
import com.boc_dev.physics_library.rigid_body_dynamics_verbose.RigidBody;

public interface Force {


	Vec3d actLinear(RigidBody rigidBody);
	Vec3d actAngular(RigidBody rigidBody);
}
