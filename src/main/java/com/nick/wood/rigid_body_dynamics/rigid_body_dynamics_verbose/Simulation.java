package com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose;

import com.nick.wood.rigid_body_dynamics.SimulationInterface;
import com.nick.wood.rigid_body_dynamics.graphics.objects.Cube;
import com.nick.wood.rigid_body_dynamics.graphics.objects.GameObject;
import com.nick.wood.rigid_body_dynamics.maths.Quaternion;
import com.nick.wood.rigid_body_dynamics.maths.Vec3d;
import com.nick.wood.rigid_body_dynamics.particle_system_dynamics_verbose.Plane;
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.ode.RigidBodyODEReturnData;
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.ode.RungeKutta;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.UUID;

public class Simulation implements SimulationInterface {

	private final RungeKutta rungeKutta;

	HashMap<UUID, RigidBodyCuboid> uuidRigidBodyCuboidHashMap = new HashMap<>();
	HashMap<UUID, GameObject> uuidGameObjectHashMap = new HashMap<>();

	public Simulation() {

		this.rungeKutta = new RungeKutta(
				(RigidBodyCuboid rigidBody) -> {

					Quaternion dDot = Quaternion.FromVec(0.0, rigidBody.getAngularVelocity()).multiply(rigidBody.getRotation()).scale(0.5);

					rigidBody.setForce(new Vec3d(0.0, 0.0, 0.0));
					rigidBody.setTorque(new Vec3d(0.0, 0.0, 1.0));

					return new RigidBodyODEReturnData(
							rigidBody.getVelocity(),
							dDot,
							rigidBody.getForce(),
							rigidBody.getTorque()
					);

				}
		);

		for (int i = 0; i < 1; i++) {
			RigidBodyCuboid rigidBody = new RigidBodyCuboid(1, new Vec3d(1.0, 2.0, 1.0), new Vec3d(i, 0.0, 0.0), new Quaternion(1.0, 0.0, 0.0, 0.0), Vec3d.ZERO, Vec3d.ZERO);
			UUID uuid = UUID.randomUUID();
			uuidRigidBodyCuboidHashMap.put(uuid, rigidBody);
			uuidGameObjectHashMap.put(uuid, convertToGameObject(rigidBody));
		}

	}

	public GameObject convertToGameObject(RigidBodyCuboid rigidBody) {
		return new GameObject(
				rigidBody.getOrigin(),
				rigidBody.getRotation().toMatrix(),
				rigidBody.getDimensions(),
				new Cube()
		);
	}

	@Override
	public void iterate(double deltaSeconds) {

		HashMap<UUID, RigidBodyCuboid> tempMap = new HashMap<>();

		uuidRigidBodyCuboidHashMap.forEach((uuid, rigidBodyCuboid) -> {
			RigidBodyCuboid rigidBody = rungeKutta.solve(rigidBodyCuboid, deltaSeconds);
			uuidGameObjectHashMap.get(uuid).setPosition(new Vec3d(rigidBody.getOrigin().getX(), rigidBody.getOrigin().getY(), rigidBody.getOrigin().getZ()));
			uuidGameObjectHashMap.get(uuid).setRotation(rigidBody.getRotation().toMatrix());
			tempMap.put(uuid, rigidBody);
		});

		uuidRigidBodyCuboidHashMap = tempMap;
	}

	@Override
	public HashMap<UUID, GameObject> getGameObjects() {
		return uuidGameObjectHashMap;
	}

	@Override
	public ArrayList<Plane> getPlanes() {
		return new ArrayList<>();
	}
}
