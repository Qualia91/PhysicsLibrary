package com.nick.wood.physics_library.rigid_body_dynamics_verbose;

import com.nick.wood.physics_library.Body;
import com.nick.wood.physics_library.SimulationInterface;
import com.nick.wood.maths.objects.QuaternionD;
import com.nick.wood.maths.objects.vector.Vec3d;
import com.nick.wood.physics_library.rigid_body_dynamics_verbose.forces.Force;
import com.nick.wood.physics_library.rigid_body_dynamics_verbose.ode.RigidBodyODEReturnData;
import com.nick.wood.physics_library.rigid_body_dynamics_verbose.ode.RungeKutta;

import java.util.*;

public class Simulation implements SimulationInterface {

	private final RungeKutta rungeKutta;
	private final CollisionDetection collisionDetection;

	ArrayList<RigidBody> rigidBodies;

	public Simulation(ArrayList<RigidBody> rigidBodies) {
		this.rigidBodies = rigidBodies;

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

		for (Force force : rigidBody.getForces()) {
			sumVecLinear = sumVecLinear.add(force.actLinear(rigidBody));
			sumVecAngular = sumVecAngular.add(force.actAngular(rigidBody));
		}

		rigidBody.addForce(sumVecLinear);
		rigidBody.addTorque(sumVecAngular);

	}

	@Override
	public void iterate(double deltaSeconds) {

		// user controls
		ArrayList<RigidBody> tempList = new ArrayList<>();

		for (RigidBody rigidBody : rigidBodies) {
			tempList.add(rungeKutta.solve(rigidBody, rigidBody.getUuid(), deltaSeconds));
		}

		rigidBodies = tempList;

		collisionDetection.collisionDetection(tempList);

		for (RigidBody rigidBody : tempList) {
			rigidBody.applyImpulse();
		}

	}

	@Override
	public ArrayList<? extends Body> getBodies() {
		return rigidBodies;
	}

}
