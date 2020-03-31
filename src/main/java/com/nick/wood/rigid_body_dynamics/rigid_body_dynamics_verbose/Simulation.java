package com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose;

import com.nick.wood.rigid_body_dynamics.SimulationInterface;
import com.nick.wood.rigid_body_dynamics.graphics.objects.GameObject;
import com.nick.wood.rigid_body_dynamics.graphics.objects.Group;
import com.nick.wood.rigid_body_dynamics.graphics.objects.Sphere;
import com.nick.wood.rigid_body_dynamics.maths.Matrix4d;
import com.nick.wood.rigid_body_dynamics.maths.Quaternion;
import com.nick.wood.rigid_body_dynamics.maths.Vec3d;
import com.nick.wood.rigid_body_dynamics.particle_system_dynamics_verbose.Plane;
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.ode.RigidBodyODEReturnData;
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.ode.RungeKutta;
import org.lwjgl.system.CallbackI;

import java.util.*;

public class Simulation implements SimulationInterface {

	private static final double Cr = 1;
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

		for (int j = 0; j < 10; j++) {
			for (int i = 0; i < 2; i++) {
				Vec3d mom = Vec3d.Y.scale(2 * i);
				if (i == 1) {
					mom = mom.neg();
				}
				RigidBody rigidBody = new RigidBody(1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(j*1.5, i * 8, 0.0), new Quaternion(1.0, 0.0, 0.0, 0.0), mom, Vec3d.Z.scale(0.1).scale(j), RigidBodyType.SPHERE);
				UUID uuid = UUID.randomUUID();
				uuidRigidBodyHashMap.put(uuid, rigidBody);
				uuidGameObjectHashMap.put(uuid, convertToGameObject(rigidBody, 10));
			}
		}

		//for (int i = 0; i < 10; i++) {
		//	Vec3d mom = Vec3d.Y.scale(10);
		//	if (i == 9) {
		//		mom = mom.neg().scale(2);
		//	}
		//	RigidBody rigidBody = new RigidBody(i, new Vec3d(1.0, 1.0, 1.0), new Vec3d(0.0, i*2, 0.0), new Quaternion(1.0, 0.0, 0.0, 0.0), mom, Vec3d.ZERO, RigidBodyType.SPHERE);
		//	UUID uuid = UUID.randomUUID();
		//	uuidRigidBodyHashMap.put(uuid, rigidBody);
		//	uuidGameObjectHashMap.put(uuid, convertToGameObject(rigidBody, 10));
		//}
		//for (int i = 0; i < 10; i++) {
		//	Vec3d mom = Vec3d.Y.scale(10).add(Vec3d.Z.scale(10));
		//	if (i == 9) {
		//		mom = mom.neg().scale(2);
		//	}
		//	RigidBody rigidBody = new RigidBody(i, new Vec3d(1.0, 1.0, 1.0), new Vec3d(0.0, i*2, 2.0), new Quaternion(1.0, 0.0, 0.0, 0.0), mom, Vec3d.ZERO, RigidBodyType.SPHERE);
		//	UUID uuid = UUID.randomUUID();
		//	uuidRigidBodyHashMap.put(uuid, rigidBody);
		//	uuidGameObjectHashMap.put(uuid, convertToGameObject(rigidBody, 10));
		//}

		//planes.add(new Plane(Vec3d.Z.scale(-10), Vec3d.Z));

	}

	private Vec3d calculateForce(RigidBody rigidBody, UUID uuid, ArrayList<Force> forces) {

		return Vec3d.ZERO;
		//return new Vec3d(0.0, 0.0, -9.81 * rigidBody.getMass());

	}

	public GameObject convertToGameObject(RigidBody rigidBody, int triangleNumber) {

		Group group = new Group();
		group.getMeshObjectArray().add(new Sphere(Vec3d.ZERO, Vec3d.ONE, Matrix4d.Identity, triangleNumber));

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
			uuidGameObjectHashMap.get(uuid).setPosition(new Vec3d(newRigidBody.getOrigin().getX(), newRigidBody.getOrigin().getY(), newRigidBody.getOrigin().getZ()));
			uuidGameObjectHashMap.get(uuid).setRotation(newRigidBody.getRotation().toMatrix());
			tempMap.put(uuid, newRigidBody);
		});

		tempMap.forEach((uuid, rigidBody) -> collisionDetection(rigidBody, tempMap, uuid));

		tempMap.forEach((uuid, rigidBody) -> rigidBody.applyImpulse());

		uuidRigidBodyHashMap = tempMap;
	}

	private void collisionDetection(RigidBody rigidBody, HashMap<UUID, RigidBody> rigidBodyMap, UUID uuid) {

		for (Map.Entry<UUID, RigidBody> uuidRigidBodyEntry : rigidBodyMap.entrySet()) {

			if (!uuidRigidBodyEntry.getKey().equals(uuid)) {

				sphereCollision(rigidBody, uuidRigidBodyEntry.getValue());

			}
		}

		// ground collision
		//for (Plane plane : planes) {
		//	// if this is bigger than 0 it hasnt collided
		//	Vec3d unitVectorFromRigidBodyToPlane = rigidBody.getOrigin().subtract(plane.getCenter());
		//	double radiusInThatDirection = rigidBody.getRotation().toMatrix().multiply(rigidBody.getDimensions()).dot(plane.getNormal())/2;
		//	double distanceFromEdgeOfSphereToPlane = unitVectorFromRigidBodyToPlane.dot(plane.getNormal()) - radiusInThatDirection;
		//	if (distanceFromEdgeOfSphereToPlane <= 0) {
		//		// calculate normal component of vector
		//		rigidBody.moveOrigin(plane.getNormal().scale(-distanceFromEdgeOfSphereToPlane));
		//		Vec3d velInDirOfNormalOfPlane = plane.getNormal().scale(plane.getNormal().dot(rigidBody.getVelocity()) * -COLLISION_CONST * 2).add(rigidBody.getVelocity());
		//		sumX += velInDirOfNormalOfPlane.getX();
		//		sumY += velInDirOfNormalOfPlane.getY();
		//		sumZ += velInDirOfNormalOfPlane.getZ();
		//		collision = true;
		//	}
		//}

	}

	private void sphereCollision(RigidBody rigidBody, RigidBody otherBody) {

		// get radius of both spheres
		// use x dimension for now
		double rigidBodyRadius = rigidBody.getDimensions().getX() / 2;
		double otherBodyRadius = otherBody.getDimensions().getX() / 2;
		double totalRadSqr = (rigidBodyRadius + otherBodyRadius);

		// get vec between the 2 centers
		Vec3d fromOtherBodyToRigid = rigidBody.getOrigin().subtract(otherBody.getOrigin());
		double length = fromOtherBodyToRigid.length();

		double collisionDist = length - totalRadSqr;


		// check if collision
		if (collisionDist < 0) {

			Vec3d nRigidBody = fromOtherBodyToRigid.normalise();
			Vec3d nOtherBody = nRigidBody.neg();

			Vec3d displacementFromCenterOfRotationToPointOfContactRigidBody = nOtherBody.scale(rigidBodyRadius);
			Vec3d displacementFromCenterOfRotationToPointOfContactOtherBody = nRigidBody.scale(otherBodyRadius);

			// diff in vel, +ve towards rigid body
			Vec3d totalVelocityRigidBody = rigidBody.getVelocity().add(rigidBody.getAngularVelocity().cross(displacementFromCenterOfRotationToPointOfContactRigidBody));
			Vec3d totalVelocityOtherBody = otherBody.getVelocity().add(otherBody.getAngularVelocity().cross(displacementFromCenterOfRotationToPointOfContactOtherBody));
			Vec3d diffInVelocitiesWithAngular = totalVelocityRigidBody.subtract(totalVelocityOtherBody);
			Vec3d diffInVelocities = rigidBody.getVelocity().subtract(otherBody.getVelocity());

			Vec3d rigidBodyPart = rigidBody.getIinv().multiply(displacementFromCenterOfRotationToPointOfContactRigidBody.cross(nRigidBody)).cross(displacementFromCenterOfRotationToPointOfContactRigidBody);
			Vec3d otherBodyPart = otherBody.getIinv().multiply(displacementFromCenterOfRotationToPointOfContactOtherBody.cross(nRigidBody)).cross(displacementFromCenterOfRotationToPointOfContactOtherBody);

			double weirdAngularPartOfJ = (rigidBodyPart.add(otherBodyPart).dot(nRigidBody));

			// calculate j
			double jLinear = -(1.0 + Cr) * diffInVelocities.dot(nRigidBody) / ((1/rigidBody.getMass() + 1/otherBody.getMass()));
			double jAngular = (-(1.0 + Cr) * diffInVelocitiesWithAngular.dot(nRigidBody)) / ((1/rigidBody.getMass() + 1/otherBody.getMass()) + weirdAngularPartOfJ);

			// calculate linear velocity impulse
			Vec3d momentumCorrectionRigidBody = nRigidBody.scale(jLinear);

			// calculate angular velocity impulse
			Vec3d angularMomentumImpulseRigidBody = displacementFromCenterOfRotationToPointOfContactRigidBody.cross(nRigidBody.cross(rigidBody.getAngularMomentum().scale(jAngular))).neg();
			Vec3d angularMomentumImpulseOtherBody = displacementFromCenterOfRotationToPointOfContactOtherBody.cross(nOtherBody.cross(otherBody.getAngularMomentum().scale(jAngular)));

			// calculate the linear momentum from angular components on collision
			Vec3d linearMomentumImpulseRigidBody = displacementFromCenterOfRotationToPointOfContactRigidBody.cross(angularMomentumImpulseOtherBody);
			Vec3d linearMomentumImpulseOtherBody = displacementFromCenterOfRotationToPointOfContactOtherBody.cross(angularMomentumImpulseRigidBody);

			// calculate the position displacement needed so they dont overlap
			Vec3d displacementVectorRigidBody = nRigidBody.scale(-collisionDist/2);

			rigidBody.addImpulse(displacementVectorRigidBody, momentumCorrectionRigidBody.add(linearMomentumImpulseRigidBody), angularMomentumImpulseRigidBody);
			rigidBody.addImpulse(Vec3d.ZERO, linearMomentumImpulseOtherBody, angularMomentumImpulseOtherBody);

		}
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
