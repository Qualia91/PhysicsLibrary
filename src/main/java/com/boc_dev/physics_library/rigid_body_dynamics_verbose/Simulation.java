package com.boc_dev.physics_library.rigid_body_dynamics_verbose;

import com.boc_dev.maths.objects.QuaternionD;
import com.boc_dev.maths.objects.vector.Vec3d;
import com.boc_dev.physics_library.rigid_body_dynamics_verbose.ode.RigidBodyODEReturnData;
import com.boc_dev.physics_library.rigid_body_dynamics_verbose.ode.RungeKutta;

import java.util.*;

public class Simulation {

	private final RungeKutta rungeKutta;
	private final CollisionDetection collisionDetection;

	public Simulation() {

		this.rungeKutta = new RungeKutta(
				(RigidBody rigidBody, UUID uuid) -> {

					QuaternionD dDot = QuaternionD.FromVec(1.0, rigidBody.getAngularVelocity()).multiply(rigidBody.getRotation()).scale(0.5);

					resolveForces(rigidBody);

					return new RigidBodyODEReturnData(
							rigidBody.getVelocity(),
							dDot,
							rigidBody.getForce(),
							rigidBody.getTorque()
					);

				}
		);

		this.collisionDetection = new CollisionDetection();
	}

	private void resolveForces(RigidBody rigidBody) {

		Vec3d sumVecLinear = Vec3d.ZERO;
		Vec3d sumVecAngular = Vec3d.ZERO;

		// todo forces need to go somewhere
		//for (Force force : rigidBody.getForces()) {
		//	sumVecLinear = sumVecLinear.add(force.actLinear(rigidBody));
		//	sumVecAngular = sumVecAngular.add(force.actAngular(rigidBody));
		//}

		rigidBody.addForce(sumVecLinear);
		rigidBody.addTorque(sumVecAngular);

	}

	public HashMap<UUID, RigidBody> iterate(double deltaSeconds, ArrayList<RigidBody> rigidBodies) {

		HashMap<UUID, RigidBody> nextIterRBs = new HashMap<>();

		for (RigidBody rigidBody : rigidBodies) {
			nextIterRBs.put(rigidBody.getUuid(), rungeKutta.solve(rigidBody, rigidBody.getUuid(), deltaSeconds));
		}

		collisionDetection.collisionDetection(new ArrayList<>(nextIterRBs.values()));

		for (RigidBody rigidBody : nextIterRBs.values()) {
			rigidBody.applyImpulse();
		}

		return nextIterRBs;
	}
}
