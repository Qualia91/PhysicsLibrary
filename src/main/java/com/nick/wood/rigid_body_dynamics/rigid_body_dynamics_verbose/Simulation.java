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
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.forces.GravityBasic;
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.ode.RigidBodyODEReturnData;
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.ode.RungeKutta;

import java.awt.geom.QuadCurve2D;
import java.util.*;

import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_RIGHT;

public class Simulation implements SimulationInterface {

	private final RungeKutta rungeKutta;
	private final Inputs input;
	private final CollisionDetection collisionDetection;
	private UUID playerRigidBodyUUID;
	private final Control control;

	ArrayList<RigidBody> rigidBodies = new ArrayList<>();
	HashMap<UUID, GameObject> uuidGameObjectHashMap = new HashMap<>();
	ArrayList<Plane> planes = new ArrayList<>();
	private double oldMouseX = 0.0;
	private double oldMouseY = 0.0;

	public Simulation(Inputs input) {

		this.input = input;

		this.control = new FlightAssistControl(0.005, 5.0);

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

		ArrayList<Force> forces = new ArrayList<>();
		//forces.add(new GravityBasic());
		//forces.add(new Drag());

		ArrayList<Force> forces2 = new ArrayList<>();
		UUID uuid = UUID.randomUUID();
		RigidBody rigidBody = new RigidBody(uuid, 1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(2.0, 0.0, 0.0), new Quaternion(1.0, 0.0, 0.0, 0.0), Vec3d.X.scale(10), Vec3d.Y.scale(0.1), RigidBodyType.SPHERE,forces);
		rigidBodies.add(rigidBody);
		uuidGameObjectHashMap.put(uuid, convertToGameObject(rigidBody, 10));

		// Arena
		UUID uuidArena = UUID.randomUUID();
		RigidBody rigidBodyArena = new RigidBody(uuidArena, 10000, new Vec3d(100, 100, 100), new Vec3d(0.0, 0.0, 0.0), new Quaternion(1.0, 0.0, 0.0, 0.0), Vec3d.ZERO, Vec3d.ZERO, RigidBodyType.SPHERE_INNER, forces);
		rigidBodies.add(rigidBodyArena);
		uuidGameObjectHashMap.put(uuidArena, convertToGameObject(rigidBodyArena, 10));

		// tests
		//for (int i = 0; i < 2; i++) {
		//	Vec3d mom = Vec3d.X.scale(i * 2);
		//	Vec3d angMom = Vec3d.Z.scale(i*0.01);
		//	if (i == 1) {
		//		mom = mom.neg();
		//		angMom = angMom.neg();
		//	}
		//	UUID uuid = UUID.randomUUID();
		//	RigidBody rigidBody = new RigidBody(uuid, 1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(i*4.0, i/2.0, 0.0), new Quaternion(1.0, 0.0, 0.0, 0.0), mom, angMom, RigidBodyType.SPHERE,forces);
		//	rigidBodies.add(rigidBody);
		//	uuidGameObjectHashMap.put(uuid, convertToGameObject(rigidBody, 10));
		//}
		//UUID uuid = UUID.randomUUID();
		//Quaternion quaternion = Quaternion.RotationX(0.0);
		//RigidBody rigidBody = new RigidBody(uuid, 1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(0.0, 0.0, 5.0), quaternion, Vec3d.ZERO, Vec3d.X.scale(0.1), RigidBodyType.CUBOID,forces);
		//rigidBodies.add(rigidBody);
		//uuidGameObjectHashMap.put(uuid, convertToGameObject(rigidBody, 10));

		//UUID uuid2 = UUID.randomUUID();
		//RigidBody rigidBody2 = new RigidBody(uuid2, 1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(0.0, -3.0, 5.0 - 0.5), new Quaternion(1.0, 0.0, 0.0, 0.0), Vec3d.Y, Vec3d.X.scale(0), RigidBodyType.SPHERE, forces);
		//rigidBodies.add(rigidBody2);
		//uuidGameObjectHashMap.put(uuid2, convertToGameObject(rigidBody2, 10));

		//UUID floorUUID = UUID.randomUUID();
		//RigidBody floorRigidBody = new RigidBody(floorUUID, 10, new Vec3d(10.0, 10.0, 1.0), new Vec3d(0.0, 0.0, -1.0), Quaternion.RotationY(0.0), Vec3d.ZERO, Vec3d.ZERO, RigidBodyType.CUBOID, forces2);
		//rigidBodies.add(floorRigidBody);
		//uuidGameObjectHashMap.put(floorUUID, convertToGameObject(floorRigidBody, 10));

		//UUID uuid3 = UUID.randomUUID();
		//RigidBody rigidBody3 = new RigidBody(uuid3, 10, new Vec3d(10.0, 10.0, 10.0), new Vec3d(0.0, 0.0, 0.0), new Quaternion(1.0, 0.0, 0.0, 0.0), Vec3d.ZERO, Vec3d.X.scale(0), RigidBodyType.SPHERE, forces);
		//rigidBodies.add(rigidBody3);
		//uuidGameObjectHashMap.put(uuid3, convertToGameObject(rigidBody3, 10));


		// cube/sphere intersection tests
		//for (int x = -1; x < 2; x++) {
		//	for (int y = -1; y < 2; y++) {
		//		for (int z = -1; z < 2; z++) {
		//			if (x == 0 && y == 0 && z == 0) {
		//				continue;
		//			}
		//			UUID uuid2 = UUID.randomUUID();
		//			RigidBody rigidBody2 = new RigidBody(uuid2, 1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(x, y, 5.0 + z), new Quaternion(1.0, 0.0, 0.0, 0.0), Vec3d.ZERO, Vec3d.ZERO, RigidBodyType.SPHERE, forces);
		//			rigidBodies.add(rigidBody2);
		//			uuidGameObjectHashMap.put(uuid2, convertToGameObject(rigidBody2, 10));
		//		}
		//	}
		//}
		//for (int i = 0; i < 2; i++) {
		//	Vec3d mom = Vec3d.Z.scale(i * 2);
		//	Vec3d angMom = Vec3d.Z.scale(i*0.1);
		//	if (i == 1) {
		//		mom = mom.neg();
		//		angMom = angMom.neg();
		//	}
		//	UUID uuid = UUID.randomUUID();
		//	RigidBody rigidBody = new RigidBody(uuid, 1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(0, i/2.0, i*4), new Quaternion(1.0, 0.0, 0.0, 0.0), mom, angMom, RigidBodyType.SPHERE,forces);
		//	rigidBodies.add(rigidBody);
		//	uuidGameObjectHashMap.put(uuid, convertToGameObject(rigidBody, 10));
		//}

		// demo 1: 2 lines interacting
		//for (int j = 0; j < 10; j++) {
		//	for (int i = 0; i < 3; i++) {
		//		Vec3d mom = Vec3d.Z.scale(i + j/10.0);// * (j/10.0));
		//		Vec3d ang = Vec3d.X.scale(0.001).scale(j);
		//		//Vec3d ang = Vec3d.ZERO;
		//		if (i > 0) {
		//			mom = mom.neg();
		//			//ang = ang.neg();
		//			//ang = Vec3d.X.scale(0.01).scale(j);
		//			ang = Vec3d.ZERO;
		//		}
		//		UUID uuidR = UUID.randomUUID();
		//		RigidBody rigidBodyR = new RigidBody(uuidR, 1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(5.0, j*3.0 - 2*i/3.0, i * 8), new Quaternion(1.0, 0.0, 0.0, 0.0), mom, ang, RigidBodyType.SPHERE,forces);
		//		rigidBodies.add(rigidBodyR);
		//		uuidGameObjectHashMap.put(uuidR, convertToGameObject(rigidBodyR, 10));
		//	}
		//}

		// demo 2: random box
		//Random random = new Random();
		//for (int k = 0; k < 10; k++) {
		//	for (int j = 0; j < 10; j++) {
		//		for (int i = 0; i < 10; i++) {
		//			UUID uuid = UUID.randomUUID();
		//			Vec3d mom = Vec3d.X.scale(random.nextInt(10) - 4).add(Vec3d.Y.scale(random.nextInt(10) - 4)).add(Vec3d.Z.scale(random.nextInt(10) - 4));
		//			Vec3d angMom = Vec3d.X.scale(random.nextInt(10) - 4).add(Vec3d.Y.scale(random.nextInt(10) - 4)).add(Vec3d.Z.scale(random.nextInt(10) - 4));
		//			RigidBody rigidBody = new RigidBody(uuid, 1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(j * 10, i * 10, k*10), new Quaternion(1.0, 0.0, 0.0, 0.0), mom, angMom.scale(0.02), RigidBodyType.SPHERE, forces);
		//			rigidBodies.add(rigidBody);
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
		RigidBody playerRigidBody = new RigidBody(playerRigidBodyUUID, 0.1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(0.0, 0.0, 0), new Quaternion(1.0, 0.0, 0.0, 0.0), Vec3d.ZERO, Vec3d.Z.scale(0.0), RigidBodyType.SPHERE, forces);
		rigidBodies.add(playerRigidBody);
		MeshGroup meshGroup = new MeshGroup();
		meshGroup.getMeshObjectArray().add(new Sphere(Vec3d.ZERO, Vec3d.ONE, Matrix4d.Identity, 10));
		PlayerGameObject playerGameObject = new PlayerGameObject(
				playerRigidBody.getOrigin(),
				playerRigidBody.getRotation().toMatrix(),
				playerRigidBody.getDimensions(),
				meshGroup
		);
		uuidGameObjectHashMap.put(playerRigidBodyUUID, playerGameObject);

		this.collisionDetection = new CollisionDetection();
	}

	private GameObject sphere(double center, double dimensions) {
		MeshGroup meshGroup = new MeshGroup();

		meshGroup.getMeshObjectArray().add(new Sphere(Vec3d.ZERO, Vec3d.ONE, Matrix4d.Identity, 11));

		return new RigidBodyGameObject(
				new Vec3d(0.0, 0.0, -center),
				Matrix4d.Identity,
				new Vec3d(dimensions, dimensions, dimensions),
				meshGroup
		);
	}

	private void resolveForces(RigidBody rigidBody) {

		Vec3d sumVec = Vec3d.ZERO;

		for (Force force : rigidBody.getForces()) {
			Vec3d act = force.act(rigidBody);
			sumVec = sumVec.add(act);
		}

		rigidBody.setForce(sumVec);
		rigidBody.setTorque(Vec3d.ZERO);

	}

	public Inputs getInputs() {
		return input;
	}

	public GameObject convertToGameObject(RigidBody rigidBody, int triangleNumber) {

		MeshGroup meshGroup = new MeshGroup();

		switch (rigidBody.getType()){
			case SPHERE_INNER:
			case SPHERE:
				meshGroup.getMeshObjectArray().add(new Sphere(Vec3d.ZERO, Vec3d.ONE, Matrix4d.Identity, triangleNumber));
				break;
			case CUBOID:
				meshGroup.getMeshObjectArray().add(new Cube(Vec3d.ZERO, Vec3d.ONE, Matrix4d.Identity));
				break;
		}

		return new RigidBodyGameObject(
				rigidBody.getOrigin(),
				rigidBody.getRotation().toMatrix(),
				rigidBody.getDimensions(),
				meshGroup
		);
	}

	@Override
	public void iterate(double deltaSeconds) {

		double newMouseX = input.getMouseX();
		double newMouseY = input.getMouseY();
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
			control.mouseMove(Math.copySign(Math.min(Math.abs(dx), 100.0), dx) , Math.copySign(Math.min(Math.abs(dy), 100.0), dy), input.isKeyPressed(GLFW_KEY_LEFT_SHIFT));
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
			for (RigidBody rigidBody : rigidBodies) {
				if (rigidBody.getUuid().equals(playerRigidBodyUUID)) {
					rigidBody.resetLinearMomentum();
					rigidBody.resetAngularMomentum();
				}
			}
		}

		// user controls
		ArrayList<RigidBody> tempList = new ArrayList<>();

		for (RigidBody rigidBody : rigidBodies) {

			if (rigidBody.getUuid().equals(playerRigidBodyUUID)) {
				rigidBody.setMomentums(control.getLinearMomentum(rigidBody.getRotation().toMatrix(), rigidBody.getLinearMomentum()), control.getAngularMomentum(rigidBody.getRotation().toMatrix(), rigidBody.getAngularMomentum()));
				control.reset();
			}
			tempList.add(rungeKutta.solve(rigidBody, rigidBody.getUuid(), deltaSeconds));
		}

		collisionDetection.collisionDetection(tempList);

		for (RigidBody rigidBody : tempList) {
			rigidBody.applyImpulse();
			uuidGameObjectHashMap.get(rigidBody.getUuid()).setPosition(rigidBody.getOrigin());
			uuidGameObjectHashMap.get(rigidBody.getUuid()).setRotation(rigidBody.getRotation().toMatrix());
		}

		rigidBodies = tempList;
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
