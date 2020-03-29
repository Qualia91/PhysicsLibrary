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

import java.lang.reflect.Array;
import java.util.*;

public class Simulation implements SimulationInterface {

	private static final double COLLISION_CONST = 0.8;
	private static final double FRICTION_CONST = 0.8;
	private final RungeKutta rungeKutta;

	HashMap<UUID, RigidBody> uuidRigidBodyHashMap = new HashMap<>();
	HashMap<UUID, GameObject> uuidGameObjectHashMap = new HashMap<>();
	ArrayList<Plane> planes = new ArrayList<>();

	public Simulation() {

		this.rungeKutta = new RungeKutta(
			(RigidBody rigidBody, UUID uuid) -> {

				Quaternion dDot = Quaternion.FromVec(0.0, rigidBody.getAngularVelocity()).multiply(rigidBody.getRotation()).scale(0.5);

				Vec3d force = calculateForce(rigidBody, uuid, new ArrayList<>());
				rigidBody.setForce(force);
				rigidBody.setTorque(new Vec3d(0.0, 0.0, 0.0));

				return new RigidBodyODEReturnData(
						rigidBody.getVelocity(),
						dDot,
						rigidBody.getForce(),
						rigidBody.getTorque()
				);

			}
		);

		//for (int i = 0; i < 1; i++) {
		//		Vec3d mom = Vec3d.Y.scale(10);
		//		if (i == 1) {
		//			mom = mom.neg();
		//		}
		//		RigidBody rigidBody = new RigidBody(1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(0.0, i*2, i/2.0), new Quaternion(1.0, 0.0, 0.0, 0.0), mom, Vec3d.ZERO, RigidBodyType.SPHERE);
		//		UUID uuid = UUID.randomUUID();
		//		uuidRigidBodyHashMap.put(uuid, rigidBody);
		//		uuidGameObjectHashMap.put(uuid, convertToGameObject(rigidBody, 10));
		//}

		for (int i = 0; i < 10; i++) {
			Vec3d mom = Vec3d.Y.scale(10);
			if (i == 9) {
				mom = mom.neg().scale(2);
			}
			RigidBody rigidBody = new RigidBody(i, new Vec3d(1.0, 1.0, 1.0), new Vec3d(0.0, i*2, 0.0), new Quaternion(1.0, 0.0, 0.0, 0.0), mom, Vec3d.ZERO, RigidBodyType.SPHERE);
			UUID uuid = UUID.randomUUID();
			uuidRigidBodyHashMap.put(uuid, rigidBody);
			uuidGameObjectHashMap.put(uuid, convertToGameObject(rigidBody, 10));
		}
		for (int i = 0; i < 10; i++) {
			Vec3d mom = Vec3d.Y.scale(10).add(Vec3d.Z.scale(10));
			if (i == 9) {
				mom = mom.neg().scale(2);
			}
			RigidBody rigidBody = new RigidBody(i, new Vec3d(1.0, 1.0, 1.0), new Vec3d(0.0, i*2, 2.0), new Quaternion(1.0, 0.0, 0.0, 0.0), mom, Vec3d.ZERO, RigidBodyType.SPHERE);
			UUID uuid = UUID.randomUUID();
			uuidRigidBodyHashMap.put(uuid, rigidBody);
			uuidGameObjectHashMap.put(uuid, convertToGameObject(rigidBody, 10));
		}

		planes.add(new Plane(Vec3d.Z.scale(-10), Vec3d.Z));

	}

	private Vec3d calculateForce(RigidBody rigidBody, UUID uuid, ArrayList<Force> forces) {

		//return Vec3d.ZERO;
		return new Vec3d(0.0, 0.0, -9.81 * rigidBody.getMass());

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
			RigidBody newRigidBody = rungeKutta.solve(rigidBody, uuid, deltaSeconds);
			collisionDetection(newRigidBody, uuidRigidBodyHashMap, uuid);
			newRigidBody.updateOrigins();
			uuidGameObjectHashMap.get(uuid).setPosition(new Vec3d(newRigidBody.getOrigin().getX(), newRigidBody.getOrigin().getY(), newRigidBody.getOrigin().getZ()));
			uuidGameObjectHashMap.get(uuid).setRotation(newRigidBody.getRotation().toMatrix());
			tempMap.put(uuid, newRigidBody);
		});

		uuidRigidBodyHashMap = tempMap;
	}

	private void collisionDetection(RigidBody rigidBody, HashMap<UUID, RigidBody> rigidBodyMap, UUID uuid) {

		boolean collision = false;
		double sumX = 0;
		double sumY = 0;
		double sumZ = 0;

		for (Map.Entry<UUID, RigidBody> uuidRigidBodyEntry : rigidBodyMap.entrySet()) {

			if (!uuidRigidBodyEntry.getKey().equals(uuid)) {

				Optional<Vec3d> sphereCollisionVector = sphereCollision(rigidBody, uuidRigidBodyEntry.getValue());

				if (sphereCollisionVector.isPresent()) {

					sumX += sphereCollisionVector.get().getX();
					sumY += sphereCollisionVector.get().getY();
					sumZ += sphereCollisionVector.get().getZ();
					collision = true;

				}

			}
		}

		// ground collision
		for (Plane plane : planes) {
			// if this is bigger than 0 it hasnt collided
			Vec3d unitVectorFromRigidBodyToPlane = rigidBody.getOrigin().subtract(plane.getCenter());
			double radiusInThatDirection = rigidBody.getRotation().toMatrix().multiply(rigidBody.getDimensions()).dot(plane.getNormal())/2;
			double distanceFromEdgeOfSphereToPlane = unitVectorFromRigidBodyToPlane.dot(plane.getNormal()) - radiusInThatDirection;
			if (distanceFromEdgeOfSphereToPlane <= 0) {
				// calculate normal component of vector
				rigidBody.moveOrigin(plane.getNormal().scale(-distanceFromEdgeOfSphereToPlane));
				Vec3d velInDirOfNormalOfPlane = plane.getNormal().scale(plane.getNormal().dot(rigidBody.getVelocity()) * -COLLISION_CONST * 2).add(rigidBody.getVelocity());
				sumX += velInDirOfNormalOfPlane.getX();
				sumY += velInDirOfNormalOfPlane.getY();
				sumZ += velInDirOfNormalOfPlane.getZ();
				collision = true;
			}
		}

		if (collision) {
			rigidBody.setVelocity(new Vec3d(sumX, sumY, sumZ));
		}

	}

	private Optional<Vec3d> sphereCollision(RigidBody rigidBody, RigidBody otherBody) {

		// get radius of both spheres
		// use x dimension for now
		double rigidBodyRadius = rigidBody.getDimensions().getX() / 2;
		double otherBodyRadius = otherBody.getDimensions().getX() / 2;
		double totalRadSqr = (rigidBodyRadius + otherBodyRadius) * (rigidBodyRadius + otherBodyRadius);

		// get vec between the 2 centers
		Vec3d fromOtherBodyToRigid = otherBody.getOrigin().subtract(rigidBody.getOrigin());
		double length2 = fromOtherBodyToRigid.length2();

		double collisionDist = length2 - totalRadSqr;

		// check if collision
		if (collisionDist < 0) {

			rigidBody.moveOrigin(fromOtherBodyToRigid.normalise().scale(collisionDist));
			otherBody.moveOrigin(fromOtherBodyToRigid.normalise().neg().scale(collisionDist));
			double massSum = rigidBody.getMass() + otherBody.getMass();
			double constant = (2 * otherBody.getMass())/(massSum);
			Vec3d v1mv2 = rigidBody.getVelocity().subtract(otherBody.getVelocity());
			Vec3d x1mx2 = rigidBody.getOrigin().subtract(otherBody.getOrigin());
			double len2 = x1mx2.length2();
			double innerProd = v1mv2.dot(x1mx2);
			double val = constant * (innerProd/len2);

			Vec3d newVel = rigidBody.getVelocity().subtract(x1mx2.scale(val));

			// collision, get impulse that needs to happen
			return Optional.of(newVel);

		}
		return Optional.empty();
	}

	@Override
	public HashMap<UUID, GameObject> getGameObjects() {
		return uuidGameObjectHashMap;
	}

	@Override
	public ArrayList<Plane> getPlanes() {
		return planes;
	}
}
