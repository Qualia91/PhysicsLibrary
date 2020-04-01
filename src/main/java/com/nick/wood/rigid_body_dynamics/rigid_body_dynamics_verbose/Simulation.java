package com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose;

import com.nick.wood.rigid_body_dynamics.SimulationInterface;
import com.nick.wood.rigid_body_dynamics.game.controls.Control;
import com.nick.wood.rigid_body_dynamics.game.controls.DirectMomentumControl;
import com.nick.wood.rigid_body_dynamics.game.controls.FlightAssistControl;
import com.nick.wood.rigid_body_dynamics.game.controls.Inputs;
import com.nick.wood.rigid_body_dynamics.game.game_objects.GameObject;
import com.nick.wood.rigid_body_dynamics.game.game_objects.PlayerGameObject;
import com.nick.wood.rigid_body_dynamics.game.game_objects.RigidBodyGameObject;
import com.nick.wood.rigid_body_dynamics.graphics.mesh_objects.*;
import com.nick.wood.rigid_body_dynamics.maths.Matrix4d;
import com.nick.wood.rigid_body_dynamics.maths.Quaternion;
import com.nick.wood.rigid_body_dynamics.maths.Vec3d;
import com.nick.wood.rigid_body_dynamics.particle_system_dynamics_verbose.Plane;
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.ode.RigidBodyODEReturnData;
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.ode.RungeKutta;

import java.util.*;

import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_RIGHT;

public class Simulation implements SimulationInterface {

	private static final double Cr = 0.5;
	private final RungeKutta rungeKutta;
	private final Inputs input;
	private final UUID playerRigidBodyUUID;
	private final Control control;

	HashMap<UUID, RigidBody> uuidRigidBodyHashMap = new HashMap<>();
	HashMap<UUID, GameObject> uuidGameObjectHashMap = new HashMap<>();
	ArrayList<Plane> planes = new ArrayList<>();
	private double newMouseX = 0.0;
	private double newMouseY = 0.0;
	private double oldMouseX = 0.0;
	private double oldMouseY = 0.0;

	public Simulation(Inputs input) {

		this.input = input;

		this.control = new FlightAssistControl(0.005, 5.0);

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

		// demo 1: 2 lines interacting
		for (int j = 0; j < 10; j++) {
			for (int i = 0; i < 2; i++) {
				Vec3d mom = Vec3d.Y.scale(4 * i);
				if (i == 1) {
					mom = mom.neg();
				}
				RigidBody rigidBody = new RigidBody(1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(j*1.5, i * 8, 0.0), new Quaternion(1.0, 0.0, 0.0, 0.0), mom, Vec3d.Z.scale(0.01).scale(j), RigidBodyType.SPHERE);
				UUID uuid = UUID.randomUUID();
				uuidRigidBodyHashMap.put(uuid, rigidBody);
				uuidGameObjectHashMap.put(uuid, convertToGameObject(rigidBody, 10));
			}
		}

		// create player
		playerRigidBodyUUID = UUID.randomUUID();
		RigidBody playerRigidBody = new RigidBody(1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(0.0, -2, 0.0), new Quaternion(1.0, 0.0, 0.0, 0.0), Vec3d.ZERO, Vec3d.Z.scale(0.0), RigidBodyType.SPHERE);
		uuidRigidBodyHashMap.put(playerRigidBodyUUID, playerRigidBody);

		MeshGroup meshGroup = new MeshGroup();
		meshGroup.getMeshObjectArray().add(new Sphere(Vec3d.ZERO, Vec3d.ONE, Matrix4d.Identity, 10));

		PlayerGameObject playerGameObject = new PlayerGameObject(
				playerRigidBody.getOrigin(),
				playerRigidBody.getRotation().toMatrix(),
				playerRigidBody.getDimensions(),
				meshGroup
		);

		uuidGameObjectHashMap.put(playerRigidBodyUUID, playerGameObject);


		// demo 2: random box
		//Random random = new Random();
		//for (int k = 0; k < 10; k++) {
		//	for (int j = 0; j < 10; j++) {
		//		for (int i = 0; i < 10; i++) {
		//			Vec3d mom = Vec3d.X.scale(random.nextInt(10) - 4).add(Vec3d.Y.scale(random.nextInt(10) - 4)).add(Vec3d.Z.scale(random.nextInt(10) - 4));
		//			Vec3d angMom = Vec3d.X.scale(random.nextInt(10) - 4).add(Vec3d.Y.scale(random.nextInt(10) - 4)).add(Vec3d.Z.scale(random.nextInt(10) - 4));
		//			RigidBody rigidBody = new RigidBody(1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(j * 10, i * 10, k*10), new Quaternion(1.0, 0.0, 0.0, 0.0), mom, angMom.scale(0.02), RigidBodyType.SPHERE);
		//			UUID uuid = UUID.randomUUID();
		//			uuidRigidBodyHashMap.put(uuid, rigidBody);
		//			uuidGameObjectHashMap.put(uuid, convertToGameObject(rigidBody, 10));
		//		}
		//	}
		//}

		// demo 3: big bang
		//Random random = new Random();
		//for (int k = -5; k < 5; k++) {
		//	for (int j = -5; j < 5; j++) {
		//		for (int i = -5; i < 5; i++) {
		//			Vec3d mom = Vec3d.X.scale(-i).add(Vec3d.Y.scale(-j)).add(Vec3d.Z.scale(-k));
		//			Vec3d angMom = Vec3d.X.scale(random.nextInt(10) - 4).add(Vec3d.Y.scale(random.nextInt(10) - 4)).add(Vec3d.Z.scale(random.nextInt(10) - 4));
		//			RigidBody rigidBody = new RigidBody(1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(i * 10, j * 10, k*10), new Quaternion(1.0, 0.0, 0.0, 0.0), mom, angMom.scale(0.02), RigidBodyType.SPHERE);
		//			UUID uuid = UUID.randomUUID();
		//			uuidRigidBodyHashMap.put(uuid, rigidBody);
		//			uuidGameObjectHashMap.put(uuid, convertToGameObject(rigidBody, 10));
		//		}
		//	}
		//}

	}

	public Inputs getInputs() {
		return input;
	}

	private Vec3d calculateForce(RigidBody rigidBody, UUID uuid, ArrayList<Force> forces) {

		return Vec3d.ZERO;
		//return new Vec3d(0.0, 0.0, -9.81 * rigidBody.getMass());

	}

	public GameObject convertToGameObject(RigidBody rigidBody, int triangleNumber) {

		MeshGroup meshGroup = new MeshGroup();
		meshGroup.getMeshObjectArray().add(new Sphere(Vec3d.ZERO, Vec3d.ONE, Matrix4d.Identity, triangleNumber));

		return new RigidBodyGameObject(
				rigidBody.getOrigin(),
				rigidBody.getRotation().toMatrix(),
				rigidBody.getDimensions(),
				meshGroup
		);
	}

	@Override
	public void iterate(double deltaSeconds) {

		newMouseX = input.getMouseX();
		newMouseY = input.getMouseY();
		if (Math.abs(oldMouseX) <= 0.000001) {
			oldMouseX = newMouseX;
		}
		if (Math.abs(oldMouseY) <= 0.000001) {
			oldMouseY = newMouseY;
		}
		double dx = newMouseX - oldMouseX;
		double dy = newMouseY - oldMouseY;
		oldMouseX = newMouseX;
		oldMouseY = newMouseY;

		if (Math.abs(dx) > 0.00001 && Math.abs(dy) > 0.00001) {
			control.mouseMove(Math.copySign(Math.min(Math.abs(dx), 100.0), dx)  , Math.copySign(Math.min(Math.abs(dy), 100.0), dy), input.isKeyPressed(GLFW_KEY_LEFT_SHIFT));
		}

		if (input.isKeyPressed(GLFW_KEY_A)) {
			control.leftLinear();
		}
		if (input.isKeyPressed(GLFW_KEY_W)) {
			control.forwardLinear();
		}
		if (input.isKeyPressed(GLFW_KEY_D)) {
			control.rightLinear();
		}
		if (input.isKeyPressed(GLFW_KEY_S)) {
			control.backLinear();
		}
		if (input.isKeyPressed(GLFW_KEY_Q)) {
			control.upLinear();
		}
		if (input.isKeyPressed(GLFW_KEY_E)) {
			control.downLinear();
		}
		if (input.isKeyPressed(GLFW_KEY_LEFT)) {
			if (input.isKeyPressed(GLFW_KEY_LEFT_SHIFT)) {
				control.leftYaw();
			} else {
				control.leftRoll();
			}
		}
		if (input.isKeyPressed(GLFW_KEY_RIGHT)) {
			if (input.isKeyPressed(GLFW_KEY_LEFT_SHIFT)) {
				control.rightYaw();
			} else {
				control.rightRoll();
			}
		}
		if (input.isKeyPressed(GLFW_KEY_UP)) {
			control.upPitch();
		}
		if (input.isKeyPressed(GLFW_KEY_DOWN)) {
			control.downPitch();
		}

		if (input.isKeyPressed(GLFW_KEY_SPACE)) {
			uuidRigidBodyHashMap.get(playerRigidBodyUUID).resetLinearMomentum();
			uuidRigidBodyHashMap.get(playerRigidBodyUUID).resetAngularMomentum();
		}

		// user controls
		HashMap<UUID, RigidBody> tempMap = new HashMap<>();

		for (Map.Entry<UUID, RigidBody> uuidRigidBodyEntry : uuidRigidBodyHashMap.entrySet()) {

			if (uuidRigidBodyEntry.getKey().equals(playerRigidBodyUUID)) {
				uuidRigidBodyEntry.getValue().setMomentums(control.getLinearMomentum(uuidRigidBodyEntry.getValue().getRotation().toMatrix(), uuidRigidBodyEntry.getValue().getLinearMomentum()), control.getAngularMomentum(uuidRigidBodyEntry.getValue().getRotation().toMatrix(), uuidRigidBodyEntry.getValue().getAngularMomentum()));

				control.reset();
			}
			tempMap.put(uuidRigidBodyEntry.getKey(), rungeKutta.solve(uuidRigidBodyEntry.getValue(), uuidRigidBodyEntry.getKey(), deltaSeconds));
		};

		tempMap.forEach((uuid, rigidBody) -> collisionDetection(rigidBody, tempMap, uuid));

		tempMap.forEach((uuid, rigidBody) -> {
			rigidBody.applyImpulse();
			uuidGameObjectHashMap.get(uuid).setPosition(rigidBody.getOrigin());
			uuidGameObjectHashMap.get(uuid).setRotation(rigidBody.getRotation().toMatrix());
		});

		uuidRigidBodyHashMap = tempMap;
	}

	private void collisionDetection(RigidBody rigidBody, HashMap<UUID, RigidBody> rigidBodyMap, UUID uuid) {

		for (Map.Entry<UUID, RigidBody> uuidRigidBodyEntry : rigidBodyMap.entrySet()) {

			if (!uuidRigidBodyEntry.getKey().equals(uuid)) {

				sphereCollision(rigidBody, uuidRigidBodyEntry.getValue());

			}
		}
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
