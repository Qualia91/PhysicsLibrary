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
import java.util.Random;
import java.util.UUID;

public class Simulation implements SimulationInterface {

	private final RungeKutta rungeKutta;
	private double stepSize = 1.0;
	private RigidBodyCuboid rigidBody = new RigidBodyCuboid(1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(0.0, 0.0, 0.0), Matrix4d.Identity, Vec3d.ZERO, Vec3d.ZERO);
	HashMap<UUID, GameObject> uuidGameObjectHashMap = new HashMap<>();
	UUID gameObUUID = UUID.randomUUID();

	public Simulation() {

		Random random = new Random();


		this.rungeKutta = new RungeKutta(
				(RigidBodyCuboid rigidBody) -> {

					Matrix4d Rdot = rigidBody.getAngularVelocity().star().multiply(rigidBody.getRotation());

					rigidBody.setForce(new Vec3d(0.0, 0.0, -1.0));
					rigidBody.setTorque(new Vec3d(0.1, 0.0, 0.0));

					return new RigidBodyODEReturnData(
							rigidBody.getVelocity(),
							Rdot,
							rigidBody.getForce(),
							rigidBody.getTorque()
					);

				}
		);

		uuidGameObjectHashMap.put(gameObUUID, convertToGameObject(rigidBody));

	}

	public GameObject convertToGameObject(RigidBodyCuboid rigidBody) {
		return new GameObject(
				new Vec3d(rigidBody.getOrigin().getX(), rigidBody.getOrigin().getZ(), rigidBody.getOrigin().getY()),
				rigidBody.getRotation(),
				new Vec3d(1.0, 1.0, 1.0),
				new Cube()
		);
	}

	@Override
	public void iterate(double deltaSeconds) {
		rigidBody = rungeKutta.solve(rigidBody, deltaSeconds);
		uuidGameObjectHashMap.get(gameObUUID).setPosition(new Vec3d(rigidBody.getOrigin().getX(), rigidBody.getOrigin().getZ(), rigidBody.getOrigin().getY()));
		uuidGameObjectHashMap.get(gameObUUID).setRotation(rigidBody.getRotation());
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
