package com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.ode;

import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.RigidBodyCuboid;

import java.util.function.Function;

public class RungeKutta {

	private final Function<RigidBodyCuboid, RigidBodyODEReturnData> ddt;

	public RungeKutta(Function<RigidBodyCuboid, RigidBodyODEReturnData> ddt) {
		this.ddt = ddt;
	}

	public RigidBodyCuboid solve(RigidBodyCuboid rigidBody, double stepSize) {

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

	private RigidBodyCuboid makeNewRigidBody(RigidBodyCuboid rigidBody, RigidBodyODEReturnData increment) {

		return rigidBody.incrementAndCopy(increment);

	}
}
