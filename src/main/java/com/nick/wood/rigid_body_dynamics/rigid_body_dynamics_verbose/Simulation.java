package com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose;

import com.nick.wood.rigid_body_dynamics.SimulationInterface;
import com.nick.wood.rigid_body_dynamics.graphics.objects.GameObject;
import com.nick.wood.rigid_body_dynamics.graphics.objects.Group;
import com.nick.wood.rigid_body_dynamics.graphics.objects.Triangle;
import com.nick.wood.rigid_body_dynamics.maths.Matrix4d;
import com.nick.wood.rigid_body_dynamics.maths.Quaternion;
import com.nick.wood.rigid_body_dynamics.maths.Vec3d;
import com.nick.wood.rigid_body_dynamics.particle_system_dynamics_verbose.Plane;
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.ode.RigidBodyODEReturnData;
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.ode.RungeKutta;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;
import java.util.UUID;

public class Simulation implements SimulationInterface {

	private final RungeKutta rungeKutta;

	HashMap<UUID, RigidBody> uuidRigidBodyHashMap = new HashMap<>();
	HashMap<UUID, GameObject> uuidGameObjectHashMap = new HashMap<>();

	public Simulation() {

		this.rungeKutta = new RungeKutta(
				(RigidBody rigidBody) -> {

					Quaternion dDot = Quaternion.FromVec(0.0, rigidBody.getAngularVelocity()).multiply(rigidBody.getRotation()).scale(0.5);

					rigidBody.setForce(new Vec3d(0.0, 0.0, 0.0));
					rigidBody.setTorque(new Vec3d(0.0, 0.0, 0.0));

					return new RigidBodyODEReturnData(
							rigidBody.getVelocity(),
							dDot,
							rigidBody.getForce(),
							rigidBody.getTorque()
					);

				}
		);

		for (int i = 0; i < 10; i++) {
			RigidBody rigidBody = new RigidBody(i+1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(0.0, i, 0.0), new Quaternion(1.0, 0.0, 0.0, 0.0), Vec3d.ZERO, Vec3d.ZERO, RigidBodyType.SPHERE);
			UUID uuid = UUID.randomUUID();
			uuidRigidBodyHashMap.put(uuid, rigidBody);
			uuidGameObjectHashMap.put(uuid, convertToGameObject(rigidBody, i+1));
		}

	}

	public GameObject convertToGameObject(RigidBody rigidBody, int triangleNumber) {

		Matrix4d mirror = Matrix4d.Rotation(180.0, Vec3d.Y);

		Triangle one = new Triangle(Vec3d.ZERO, Vec3d.ONE, Matrix4d.Identity, triangleNumber);
		Triangle two = new Triangle(Vec3d.ZERO, Vec3d.ONE, Matrix4d.Rotation(90.0, Vec3d.Z), triangleNumber);
		Triangle three = new Triangle(Vec3d.ZERO, Vec3d.ONE, Matrix4d.Rotation(180.0, Vec3d.Z), triangleNumber);
		Triangle four = new Triangle(Vec3d.ZERO, Vec3d.ONE, Matrix4d.Rotation(270.0, Vec3d.Z), triangleNumber);
		Triangle five = new Triangle(Vec3d.ZERO, Vec3d.ONE, mirror, triangleNumber);
		Triangle six = new Triangle(Vec3d.ZERO, Vec3d.ONE, mirror.multiply(Matrix4d.Rotation(90.0, Vec3d.Z)), triangleNumber);
		Triangle seven = new Triangle(Vec3d.ZERO, Vec3d.ONE, mirror.multiply(Matrix4d.Rotation(180.0, Vec3d.Z)), triangleNumber);
		Triangle eight = new Triangle(Vec3d.ZERO, Vec3d.ONE, mirror.multiply(Matrix4d.Rotation(270.0, Vec3d.Z)), triangleNumber);

		Group group = new Group();
		group.getMeshObjectArray().add(one);
		group.getMeshObjectArray().add(two);
		group.getMeshObjectArray().add(three);
		group.getMeshObjectArray().add(four);
		group.getMeshObjectArray().add(five);
		group.getMeshObjectArray().add(six);
		group.getMeshObjectArray().add(seven);
		group.getMeshObjectArray().add(eight);

		return new GameObject(
				rigidBody.getOrigin(),
				rigidBody.getRotation().toMatrix(),
				rigidBody.getDimensions(),
				group
		);
	}

	@Override
	public void iterate(double deltaSeconds) {

		HashMap<UUID, RigidBody> tempMap = new HashMap<>();

		uuidRigidBodyHashMap.forEach((uuid, rigidBody) -> {
			RigidBody newRigidBody = rungeKutta.solve(rigidBody, deltaSeconds);
			collisionDetection(newRigidBody, uuidRigidBodyHashMap);
			uuidGameObjectHashMap.get(uuid).setPosition(new Vec3d(newRigidBody.getOrigin().getX(), newRigidBody.getOrigin().getY(), newRigidBody.getOrigin().getZ()));
			uuidGameObjectHashMap.get(uuid).setRotation(newRigidBody.getRotation().toMatrix());
			tempMap.put(uuid, newRigidBody);
		});

		uuidRigidBodyHashMap = tempMap;
	}

	private void collisionDetection(RigidBody rigidBody, HashMap<UUID, RigidBody> rigidBodyMap) {

		for (RigidBody otherBody : rigidBodyMap.values()) {

			Optional<Vec3d> sphereCollisionVector = sphereCollision(rigidBody, otherBody);

			if (sphereCollisionVector.isPresent()) {



			}

		}

	}

	private Optional<Vec3d> sphereCollision(RigidBody rigidBody, RigidBody otherBody) {
		return null;
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
