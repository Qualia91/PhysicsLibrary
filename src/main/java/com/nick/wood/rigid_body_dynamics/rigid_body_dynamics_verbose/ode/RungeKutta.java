package com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.ode;

import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.RigidBody;

import java.util.function.Function;

public class RungeKutta {

	private final Function<RigidBody, RigidBodyODEReturnData> ddt;

	public RungeKutta(Function<RigidBody, RigidBodyODEReturnData> ddt) {
		this.ddt = ddt;
	}

	public RigidBody solve(RigidBody rigidBody, double stepSize) {

		// K1
		RigidBodyODEReturnData k1 = ddt.apply(rigidBody);

		// K2
		RigidBodyODEReturnData k2 = ddt.apply(makeNewRigidBody(rigidBody, k1.scale(stepSize/2)));

		// K3
		RigidBodyODEReturnData k3 = ddt.apply(makeNewRigidBody(rigidBody, k2.scale(stepSize/2)));

		// K4
		RigidBodyODEReturnData k4 = ddt.apply(makeNewRigidBody(rigidBody, k3.scale(stepSize)));

		// step
		return makeNewRigidBody(rigidBody, ((k1.add(k2.scale(2)).add(k3.scale(2)).add(k4)).scale(1.0/6.0).scale(stepSize)));

	}

	private RigidBody makeNewRigidBody(RigidBody rigidBody, RigidBodyODEReturnData increment) {

		return rigidBody.incrementAndCopy(increment);

	}
}
