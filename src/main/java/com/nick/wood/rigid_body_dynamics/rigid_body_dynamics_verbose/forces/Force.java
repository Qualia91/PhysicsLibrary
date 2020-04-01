package com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.forces;

import com.nick.wood.rigid_body_dynamics.maths.Vec3d;
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.RigidBody;

public interface Force {


	Vec3d act(RigidBody rigidBody, RigidBody otherRigidBody);
}
