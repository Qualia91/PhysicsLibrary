package com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose;

import com.nick.wood.graphics_library.objects.game_objects.*;
import com.nick.wood.maths.objects.vector.Vec3f;
import com.nick.wood.rigid_body_dynamics.SimulationInterface;
import com.nick.wood.rigid_body_dynamics.game.controls.FlightAssistControl;
import com.nick.wood.graphics_library.input.*;
import com.nick.wood.maths.objects.Quaternion;
import com.nick.wood.maths.objects.vector.Vec3d;
import com.nick.wood.rigid_body_dynamics.particle_system_dynamics_verbose.Plane;
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.forces.Force;
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.ode.RigidBodyODEReturnData;
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.ode.RungeKutta;

import java.util.*;

public class Simulation implements SimulationInterface {

	private final RungeKutta rungeKutta;
	private final Inputs input;
	private final CollisionDetection collisionDetection;
	private final Game3DInputs game3DInputs;
	private UUID playerRigidBodyUUID;
	private final Control control;

	ArrayList<RigidBody> rigidBodies;
	HashMap<UUID, RootGameObject> rootGameObjectHashMap = new HashMap<>();
	ArrayList<Plane> planes = new ArrayList<>();

	public Simulation(Inputs input, ArrayList<RigidBody> rigidBodies, HashMap<UUID, RootGameObject> rootGameObjectHashMap) {
		this.rigidBodies = rigidBodies;
		this.input = input;

		this.control = new FlightAssistControl(0.005, 5.0);
		this.game3DInputs = new Game3DInputs(input, control);

		this.rungeKutta = new RungeKutta(
				(RigidBody rigidBody, UUID uuid) -> {

					Quaternion dDot = Quaternion.FromVec(0.0, rigidBody.getAngularVelocity()).multiply(rigidBody.getRotation()).scale(0.5);

					resolveForces(rigidBody);

					return new RigidBodyODEReturnData(
							rigidBody.getVelocity(),
							dDot,
							rigidBody.getForce(),
							rigidBody.getTorque()
					);

				}
		);

		this.rootGameObjectHashMap = rootGameObjectHashMap;

		//for (RigidBody rigidBody : rigidBodies) {
		//	rootGameObjectHashMap.put(rigidBody.getUuid(), convertToGameObject(rigidBody));
		//}
		//rootGameObjectHashMap.put(UUID.randomUUID(), cameraGameObject);
		//rootGameObjectHashMap.put(UUID.randomUUID(), lightRootObject);

		this.collisionDetection = new CollisionDetection();
	}

	private void resolveForces(RigidBody rigidBody) {

		Vec3d sumVecLinear = Vec3d.ZERO;
		Vec3d sumVecAngular = Vec3d.ZERO;

		for (Force force : rigidBody.getForces()) {
			sumVecLinear = sumVecLinear.add(force.actLinear(rigidBody));
			sumVecAngular = sumVecAngular.add(force.actAngular(rigidBody));
		}

		rigidBody.setForce(sumVecLinear);
		rigidBody.setTorque(sumVecAngular);

	}

	public Inputs getInputs() {
		return input;
	}

	@Override
	public HashMap<UUID, RootGameObject> getRootGameObjects() {
		return rootGameObjectHashMap;
	}

	@Override
	public void iterate(double deltaSeconds) {

		// user controls
		ArrayList<RigidBody> tempList = new ArrayList<>();

		for (RigidBody rigidBody : rigidBodies) {

			if (rigidBody.getUuid().equals(playerRigidBodyUUID)) {
				rigidBody.setMomentums(control.getLinearMomentum(rigidBody.getRotation().toMatrix(), rigidBody.getLinearMomentum()), control.getAngularMomentum(rigidBody.getRotation().toMatrix(), rigidBody.getAngularMomentum()));
				control.reset();
			}
			tempList.add(rungeKutta.solve(rigidBody, rigidBody.getUuid(), deltaSeconds));
		}

		rigidBodies = tempList;

		collisionDetection.collisionDetection(tempList);

		game3DInputs.checkInputs();

		for (RigidBody rigidBody : tempList) {
			rigidBody.applyImpulse();
		}

		mapToGameObjects(rootGameObjectHashMap, rigidBodies);

	}

	private static void mapToGameObjects(HashMap<UUID, RootGameObject> gameObjects, ArrayList<RigidBody> rigidBodies) {

		for (RigidBody rigidBody : rigidBodies) {

			TransformGameObject transformGameObject = (TransformGameObject) gameObjects.get(rigidBody.getUuid()).getGameObjectNodeData().getChildren().get(0);
			transformGameObject.setPosition((Vec3f) rigidBody.getOrigin().toVecf());
			transformGameObject.setRotation(rigidBody.getRotation().toMatrix().toMatrix4f());

		}

	}

	@Override
	public ArrayList<Plane> getPlanes() {
		return planes;
	}
}
