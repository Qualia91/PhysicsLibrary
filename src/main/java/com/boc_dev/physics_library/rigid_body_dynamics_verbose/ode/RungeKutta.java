package com.boc_dev.physics_library.rigid_body_dynamics_verbose.ode;

import com.boc_dev.physics_library.rigid_body_dynamics_verbose.RigidBody;
import com.boc_dev.physics_library.rigid_body_dynamics_verbose.forces.DDTInterface;

import java.util.ArrayList;
import java.util.UUID;
import java.util.function.BiFunction;

public class RungeKutta {

	private final DDTInterface ddt;

	public RungeKutta(DDTInterface ddt) {
		this.ddt = ddt;
	}

	public RigidBody solve(RigidBody rigidBody, UUID uuid, double stepSize, ArrayList<RigidBody> rigidBodies) {

		// K1
		RigidBodyODEReturnData k1 = ddt.apply(rigidBody, uuid, rigidBodies);

		// K2
		RigidBodyODEReturnData k2 = ddt.apply(makeNewRigidBody(rigidBody, k1.scale(stepSize/2)), uuid, rigidBodies);

		// K3
		RigidBodyODEReturnData k3 = ddt.apply(makeNewRigidBody(rigidBody, k2.scale(stepSize/2)), uuid, rigidBodies);

		// K4
		RigidBodyODEReturnData k4 = ddt.apply(makeNewRigidBody(rigidBody, k3.scale(stepSize)), uuid, rigidBodies);

		// step
		return makeNewRigidBody(rigidBody, ((k1.add(k2.scale(2)).add(k3.scale(2)).add(k4)).scale(1.0/6.0).scale(stepSize)));

	}

	private RigidBody makeNewRigidBody(RigidBody rigidBody, RigidBodyODEReturnData increment) {

		return rigidBody.incrementAndCopy(increment);

	}
}
