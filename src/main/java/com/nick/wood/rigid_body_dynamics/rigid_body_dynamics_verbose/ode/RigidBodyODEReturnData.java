package com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.ode;

import com.nick.wood.rigid_body_dynamics.maths.Matrix4d;
import com.nick.wood.rigid_body_dynamics.maths.Vec3d;

public class RigidBodyODEReturnData {

	public Vec3d Xdot;
	public Matrix4d Rdot;
	public Vec3d Pdot;
	public Vec3d Ldot;

	public RigidBodyODEReturnData(Vec3d xdot, Matrix4d rdot, Vec3d pdot, Vec3d ldot) {
		Xdot = xdot;
		Rdot = rdot;
		Pdot = pdot;
		Ldot = ldot;
	}

	public RigidBodyODEReturnData scale(double amount) {
		Vec3d newXdot = Xdot.scale(amount);
		Matrix4d newRdot = Rdot.scale(amount);
		Vec3d newPdot = Pdot.scale(amount);
		Vec3d newLdot = Ldot.scale(amount);
		return new RigidBodyODEReturnData(newXdot, newRdot, newPdot, newLdot);
	}

	public RigidBodyODEReturnData add(RigidBodyODEReturnData rigidBodyODEReturnData) {
		return new RigidBodyODEReturnData(
				this.Xdot.add(rigidBodyODEReturnData.Xdot),
				this.Rdot.add(rigidBodyODEReturnData.Rdot),
				this.Pdot.add(rigidBodyODEReturnData.Pdot),
				this.Ldot.add(rigidBodyODEReturnData.Ldot)
		);
	}
}
