package com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose;

import com.nick.wood.rigid_body_dynamics.SimulationInterface;
import com.nick.wood.rigid_body_dynamics.game.controls.Control;
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
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.forces.Force;
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.forces.Gravity;
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.ode.RigidBodyODEReturnData;
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.ode.RungeKutta;

import java.util.*;

import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_RIGHT;

public class Simulation implements SimulationInterface {

	private static final double Cr = 0.5;
	private static final double ANGULAR_MOMENTUM_SPLIT = 0.5;
	private static final double FRICTION = 0.12;
	private final RungeKutta rungeKutta;
	private final Inputs input;
	private UUID playerRigidBodyUUID;
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

				resolveForces(rigidBody, uuid);

				return new RigidBodyODEReturnData(
						rigidBody.getVelocity(),
						dDot,
						rigidBody.getForce(),
						rigidBody.getTorque()
				);

			}
		);

		ArrayList<Force> forces = new ArrayList<>();
		forces.add(new Gravity());

		for (int i = 0; i < 2; i++) {
			Vec3d mom = Vec3d.X.scale(i * 2);
			Vec3d angMom = Vec3d.Z.scale(i*0.1);
			if (i == 1) {
				mom = mom.neg();
				angMom = angMom.neg();
			}
			RigidBody rigidBody = new RigidBody(1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(i * 4, 0.0, 10 + i/2.0), new Quaternion(1.0, 0.0, 0.0, 0.0), mom, angMom, RigidBodyType.SPHERE,forces);
			UUID uuid = UUID.randomUUID();
			uuidRigidBodyHashMap.put(uuid, rigidBody);
			uuidGameObjectHashMap.put(uuid, convertToGameObject(rigidBody, 10));
		}

		// demo 1: 2 lines interacting
		//for (int j = 0; j < 10; j++) {
		//	for (int i = 0; i < 2; i++) {
		//		Vec3d mom = Vec3d.Y.scale(4 * i);
		//		if (i == 1) {
		//			mom = mom.neg();
		//		}
		//		RigidBody rigidBody = new RigidBody(1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(j*1.5, i * 8, 10), new Quaternion(1.0, 0.0, 0.0, 0.0), mom, Vec3d.Z.scale(0.01).scale(j), RigidBodyType.SPHERE,forces);
		//		UUID uuid = UUID.randomUUID();
		//		uuidRigidBodyHashMap.put(uuid, rigidBody);
		//		uuidGameObjectHashMap.put(uuid, convertToGameObject(rigidBody, 10));
		//	}
		//}


		// demo 2: random box
		//Random random = new Random();
		//for (int k = 0; k < 10; k++) {
		//	for (int j = 0; j < 10; j++) {
		//		for (int i = 0; i < 10; i++) {
		//			Vec3d mom = Vec3d.X.scale(random.nextInt(10) - 4).add(Vec3d.Y.scale(random.nextInt(10) - 4)).add(Vec3d.Z.scale(random.nextInt(10) - 4));
		//			Vec3d angMom = Vec3d.X.scale(random.nextInt(10) - 4).add(Vec3d.Y.scale(random.nextInt(10) - 4)).add(Vec3d.Z.scale(random.nextInt(10) - 4));
		//			RigidBody rigidBody = new RigidBody(1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(j * 10, i * 10, k*10), new Quaternion(1.0, 0.0, 0.0, 0.0), mom, angMom.scale(0.02), RigidBodyType.SPHERE, forces);
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

		// create player
		playerRigidBodyUUID = UUID.randomUUID();
		RigidBody playerRigidBody = new RigidBody(1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(-5.0, 0.0, 10), new Quaternion(1.0, 0.0, 0.0, 0.0), Vec3d.ZERO, Vec3d.Z.scale(0.0), RigidBodyType.SPHERE, forces);
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
	}

	private void resolveForces(RigidBody rigidBody, UUID uuid) {

		Vec3d sumVec = Vec3d.ZERO;

		for (Map.Entry<UUID, RigidBody> uuidRigidBodyEntry : uuidRigidBodyHashMap.entrySet()) {
			if (!uuid.equals(uuidRigidBodyEntry.getKey())) {
				for (Force force : rigidBody.getForces()) {
					Vec3d act = force.act(rigidBody, uuidRigidBodyEntry.getValue());
					sumVec = sumVec.add(act);
				}
			}
		}

		rigidBody.setForce(sumVec);
		rigidBody.setTorque(Vec3d.ZERO);

	}

	public Inputs getInputs() {
		return input;
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

		Vec3d totLinearMomentum = Vec3d.ZERO;
		Vec3d totAngularMomentum = Vec3d.ZERO;

		for (Map.Entry<UUID, RigidBody> uuidRigidBodyEntry : tempMap.entrySet()) {
			uuidRigidBodyEntry.getValue().applyImpulse();
			totLinearMomentum = totLinearMomentum.add(uuidRigidBodyEntry.getValue().getLinearMomentum());
			totAngularMomentum = totAngularMomentum.add(uuidRigidBodyEntry.getValue().getAngularMomentum());
			uuidGameObjectHashMap.get(uuidRigidBodyEntry.getKey()).setPosition(uuidRigidBodyEntry.getValue().getOrigin());
			uuidGameObjectHashMap.get(uuidRigidBodyEntry.getKey()).setRotation(uuidRigidBodyEntry.getValue().getRotation().toMatrix());
		}

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
			Vec3d diffInVelocitiesWithAngular = totalVelocityRigidBody.subtract(otherBody.getVelocity());

			Vec3d rigidBodyPart = rigidBody.getIinv().multiply(displacementFromCenterOfRotationToPointOfContactRigidBody.cross(nRigidBody)).cross(displacementFromCenterOfRotationToPointOfContactRigidBody);
			Vec3d otherBodyPart = otherBody.getIinv().multiply(displacementFromCenterOfRotationToPointOfContactOtherBody.cross(nRigidBody)).cross(displacementFromCenterOfRotationToPointOfContactOtherBody);

			double weirdAngularPartOfJ = (rigidBodyPart.add(otherBodyPart).dot(nRigidBody));

			Vec3d t = nRigidBody.cross(diffInVelocitiesWithAngular).cross(nRigidBody).normalise();

			// calculate j
			double jAngular = (-(1.0 + Cr) * diffInVelocitiesWithAngular.dot(nRigidBody)) / ((1/rigidBody.getMass() + 1/otherBody.getMass()) + weirdAngularPartOfJ);

			// calculate linear momentum impulse due to collision
			Vec3d momentumCorrectionRigidBody = nRigidBody.subtract(t.scale(FRICTION)).scale(jAngular / rigidBody.getMass());

			// calculate angular momentum impulse due to collision
			Vec3d angularMomentumImpulseRigidBody = displacementFromCenterOfRotationToPointOfContactOtherBody.cross(nRigidBody.add(t.scale(FRICTION)).scale(jAngular));

			// calculate the position displacement needed so they dont overlap
			Vec3d displacementVectorRigidBody = nRigidBody.scale(-collisionDist/2);

			rigidBody.addImpulse(displacementVectorRigidBody, momentumCorrectionRigidBody, angularMomentumImpulseRigidBody);

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
