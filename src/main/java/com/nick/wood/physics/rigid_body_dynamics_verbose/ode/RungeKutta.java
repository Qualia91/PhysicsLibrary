package com.nick.wood.physics.rigid_body_dynamics_verbose.ode;

import com.nick.wood.physics.rigid_body_dynamics_verbose.RigidBody;

import java.util.UUID;
import java.util.function.BiFunction;

public class RungeKutta {

	private final BiFunction<RigidBody, UUID, RigidBodyODEReturnData> ddt;

	public RungeKutta(BiFunction<RigidBody, UUID, RigidBodyODEReturnData> ddt) {
		this.ddt = ddt;
	}

	public RigidBody solve(RigidBody rigidBody, UUID uuid, double stepSize) {

		// K1
		RigidBodyODEReturnData k1 = ddt.apply(rigidBody, uuid);

		// K2
		RigidBodyODEReturnData k2 = ddt.apply(makeNewRigidBody(rigidBody, k1.scale(stepSize/2)), uuid);

		// K3
		RigidBodyODEReturnData k3 = ddt.apply(makeNewRigidBody(rigidBody, k2.scale(stepSize/2)), uuid);

		// K4
		RigidBodyODEReturnData k4 = ddt.apply(makeNewRigidBody(rigidBody, k3.scale(stepSize)), uuid);

		// step
		return makeNewRigidBody(rigidBody, ((k1.add(k2.scale(2)).add(k3.scale(2)).add(k4)).scale(1.0/6.0).scale(stepSize)));

	}

	private RigidBody makeNewRigidBody(RigidBody rigidBody, RigidBodyODEReturnData increment) {

		return rigidBody.incrementAndCopy(increment);

	}
}
