package com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.ode;

import com.nick.wood.rigid_body_dynamics.maths.Quaternion;
import com.nick.wood.rigid_body_dynamics.maths.Vec3d;

public class RigidBodyODEReturnData {

	public Vec3d Xdot;
	public Quaternion Qdot;
	public Vec3d Pdot;
	public Vec3d Ldot;

	public RigidBodyODEReturnData(Vec3d xdot, Quaternion qdot, Vec3d pdot, Vec3d ldot) {
		this.Xdot = xdot;
		this.Qdot = qdot;
		this.Pdot = pdot;
		this.Ldot = ldot;
	}

	public RigidBodyODEReturnData scale(double amount) {
		Vec3d newXdot = Xdot.scale(amount);
		Quaternion newQdot = Qdot.scale(amount);
		Vec3d newPdot = Pdot.scale(amount);
		Vec3d newLdot = Ldot.scale(amount);
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
