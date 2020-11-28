package com.boc_dev.physics_library.rigid_body_dynamics_verbose.forces;

import com.boc_dev.physics_library.rigid_body_dynamics_verbose.RigidBody;
import com.boc_dev.physics_library.rigid_body_dynamics_verbose.ode.RigidBodyODEReturnData;

import java.util.ArrayList;
import java.util.UUID;

public interface DDTInterface {
	RigidBodyODEReturnData apply(RigidBody rigidBody, UUID uuid, ArrayList<RigidBody> rigidBodies);
}
