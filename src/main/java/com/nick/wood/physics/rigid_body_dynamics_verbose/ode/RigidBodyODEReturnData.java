package com.nick.wood.physics.rigid_body_dynamics_verbose.ode;

import com.nick.wood.maths.objects.QuaternionD;
import com.nick.wood.maths.objects.vector.Vecd;

public class RigidBodyODEReturnData {

	public Vecd Xdot;
	public QuaternionD Qdot;
	public Vecd Pdot;
	public Vecd Ldot;

	public RigidBodyODEReturnData(Vecd xdot, QuaternionD qdot, Vecd pdot, Vecd ldot) {
		this.Xdot = xdot;
		this.Qdot = qdot;
		this.Pdot = pdot;
		this.Ldot = ldot;
	}

	public RigidBodyODEReturnData scale(double amount) {
		Vecd newXdot = Xdot.scale(amount);
		QuaternionD newQdot = Qdot.scale(amount);
		Vecd newPdot = Pdot.scale(amount);
		Vecd newLdot = Ldot.scale(amount);
		return new RigidBodyODEReturnData(newXdot, newQdot, newPdot, newLdot);
	}

	public RigidBodyODEReturnData add(RigidBodyODEReturnData rigidBodyODEReturnData) {
		return new RigidBodyODEReturnData(
				this.Xdot.add(rigidBodyODEReturnData.Xdot),
				this.Qdot.add(rigidBodyODEReturnData.Qdot),
				this.Pdot.add(rigidBodyODEReturnData.Pdot),
				this.Ldot.add(rigidBodyODEReturnData.Ldot)
		);
	}
}
