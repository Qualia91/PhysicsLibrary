package com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose;

import com.nick.wood.rigid_body_dynamics.SimulationInterface;
import com.nick.wood.rigid_body_dynamics.graphics.objects.Cube;
import com.nick.wood.rigid_body_dynamics.graphics.objects.GameObject;
import com.nick.wood.rigid_body_dynamics.maths.Matrix4d;
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

					Matrix4d Rdot = rigidBody.getAngularVelocity().star().multiply(rigidBody.getRotation());

					rigidBody.setForce(new Vec3d(0.0, 0.0, 0.0));
					rigidBody.setTorque(new Vec3d(0.0, 0.0, 0.0));

					return new RigidBodyODEReturnData(
							rigidBody.getVelocity(),
							Rdot,
							rigidBody.getForce(),
							rigidBody.getTorque()
					);

				}
		);

		for (int i = 0; i < 1; i++) {
			RigidBodyCuboid rigidBody = new RigidBodyCuboid(1, new Vec3d(2.0, 1.0, 1.0), new Vec3d(i, 0.0, 0.0), Matrix4d.Identity, Vec3d.ZERO, Vec3d.ZERO);
			UUID uuid = UUID.randomUUID();
			uuidRigidBodyCuboidHashMap.put(uuid, rigidBody);
			uuidGameObjectHashMap.put(uuid, convertToGameObject(rigidBody));
		}

	}

	public GameObject convertToGameObject(RigidBodyCuboid rigidBody) {
		return new GameObject(
				rigidBody.getOrigin(),
				rigidBody.getRotation(),
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
			uuidGameObjectHashMap.get(uuid).setRotation(rigidBody.getRotation());
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
