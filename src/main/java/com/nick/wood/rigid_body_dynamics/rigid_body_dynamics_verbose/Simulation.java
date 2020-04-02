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
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.ode.RigidBodyODEReturnData;
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.ode.RungeKutta;

import java.util.*;

import static org.lwjgl.glfw.GLFW.*;
import static org.lwjgl.glfw.GLFW.GLFW_KEY_RIGHT;

public class Simulation implements SimulationInterface {

	private static final double Cr = 1;
	private static final double ANGULAR_MOMENTUM_LOST = 0.5;
	private static final double FRICTION = 0.1;
	private final RungeKutta rungeKutta;
	private final Inputs input;
	private UUID playerRigidBodyUUID;
	private final Control control;

	ArrayList<RigidBody> rigidBodies = new ArrayList<>();
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
		//forces.add(new Gravity());
		//forces.add(new Drag());

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
		//for (int j = 0; j < 15; j++) {
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
		//		UUID uuid = UUID.randomUUID();
		//		RigidBody rigidBody = new RigidBody(uuid, 1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(5.0, j*3.0 - 2*i/3.0, i * 8), new Quaternion(1.0, 0.0, 0.0, 0.0), mom, ang, RigidBodyType.SPHERE,forces);
		//		rigidBodies.add(rigidBody);
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
		RigidBody playerRigidBody = new RigidBody(playerRigidBodyUUID, 1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(-5.0, 0.0, 3), new Quaternion(1.0, 0.0, 0.0, 0.0), Vec3d.ZERO, Vec3d.Z.scale(0.0), RigidBodyType.SPHERE, forces);
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
	}

	private void resolveForces(RigidBody rigidBody, UUID uuid) {

		Vec3d sumVec = Vec3d.ZERO;

		for (RigidBody otherRigidBody : rigidBodies) {
			if (!uuid.equals(otherRigidBody.getUuid())) {
				for (Force force : rigidBody.getForces()) {
					Vec3d act = force.act(rigidBody, otherRigidBody);
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
			for (RigidBody rigidBody : rigidBodies) {
				if (rigidBody.getUuid().equals(playerRigidBodyUUID)) {
					rigidBody.resetLinearMomentum();
					rigidBody.resetAngularMomentum();
				}
			}
		}

		// user controls
		ArrayList<RigidBody> tempList= new ArrayList<>();

		for (RigidBody rigidBody : rigidBodies) {

			if (rigidBody.getUuid().equals(playerRigidBodyUUID)) {
				rigidBody.setMomentums(control.getLinearMomentum(rigidBody.getRotation().toMatrix(), rigidBody.getLinearMomentum()), control.getAngularMomentum(rigidBody.getRotation().toMatrix(), rigidBody.getAngularMomentum()));
				control.reset();
			}
			tempList.add(rungeKutta.solve(rigidBody, rigidBody.getUuid(), deltaSeconds));
		};

		collisionDetection(tempList);

		Vec3d totLinearMomentum = Vec3d.ZERO;
		Vec3d totAngularMomentum = Vec3d.ZERO;

		for (RigidBody rigidBody : tempList) {
			rigidBody.applyImpulse();
			totLinearMomentum = totLinearMomentum.add(rigidBody.getLinearMomentum());
			totAngularMomentum = totAngularMomentum.add(rigidBody.getAngularMomentum());
			uuidGameObjectHashMap.get(rigidBody.getUuid()).setPosition(rigidBody.getOrigin());
			uuidGameObjectHashMap.get(rigidBody.getUuid()).setRotation(rigidBody.getRotation().toMatrix());
		}

		rigidBodies = tempList;
	}

	private void collisionDetection(ArrayList<RigidBody> rigidBodyList) {

		for (int i = 0; i < rigidBodyList.size() - 1; i++) {
			for (int j = i+1; j < rigidBodyList.size(); j++) {
		//for (int i = 0; i < rigidBodyList.size(); i++) {
		//	for (int j = 0; j < rigidBodyList.size(); j++) {
				if (i != j) {
					sphereCollision(rigidBodyList.get(i), rigidBodyList.get(j));
				}
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

			// this is all the maths that will work even if they aren't spheres
			// for spheres, most of it wont do anything

			double ma = rigidBody.getMass();
			double mb = otherBody.getMass();
			// normal of impact point on other body
			Vec3d n = fromOtherBodyToRigid.normalise();

			// distance vec from center of mass of A to point of impact
			Vec3d rap = n.neg().scale(rigidBodyRadius);
			// distance vec from center of mass of B to point of impact
			Vec3d rbp = n.scale(rigidBodyRadius);

			// initial angular velocities
			Vec3d wa1 = rigidBody.getAngularVelocity();
			Vec3d wb1 = otherBody.getAngularVelocity();

			// initial velocities
			Vec3d va1 = rigidBody.getVelocity();
			Vec3d vb1 = otherBody.getVelocity();

			// initial velocity of impact point on A
			Vec3d vap1 = va1.add(wa1.cross(rap));
			// initial velocity of impact point on B
			Vec3d vbp1 = vb1.add(wb1.cross(rbp));

			// initial relative velocity of point on A
			Vec3d vabDiff = va1.subtract(vb1);
			// initial relative velocity of point on B
			Vec3d vbaDiff = vb1.subtract(va1);

			// initial relative velocity of point on A
			Vec3d vab1 = vap1.subtract(vbp1);
			// initial relative velocity of point on B
			Vec3d vba1 = vbp1.subtract(vap1);

			// above values but in the direction of n
			Vec3d vab1n = n.scale(vab1.dot(n));
			Vec3d vba1n = n.scale(vba1.dot(n));

			// initial relative angular velocity of point on A
			Vec3d wab1 = wa1.subtract(wb1);
			// initial relative velocity of point on B
			Vec3d wba1 = wb1.subtract(wa1);

			// intertal tensor inverse
			Matrix4d Ia = rigidBody.getInertialTensor();
			Matrix4d Ib = otherBody.getInertialTensor();
			Matrix4d IaInv = rigidBody.getIinv();
			Matrix4d IbInv = otherBody.getIinv();

			// impulse param j
			double jal = -(1.0+Cr) * vab1.dot(n) / (
					(1.0/ma) +
							(1.0/mb)
			);
			double jbl = -(1.0+Cr) * vba1.dot(n) / (
					(1.0/ma) +
							(1.0/mb)
			);
			double ja = -(1.0+Cr) * vabDiff.dot(n) / (
					(1.0/ma) +
							(1.0/mb) +
							(IaInv.multiply(rap.cross(n)).cross(rap)).add((IbInv.multiply(rbp.cross(n)).cross(rbp))).dot(n)
			);
			double jb = -(1.0+Cr) * vbaDiff.dot(n) / (
					(1.0/ma) +
							(1.0/mb) +
							(IaInv.multiply(rbp.cross(n)).cross(rbp)).add((IbInv.multiply(rap.cross(n)).cross(rap))).dot(n)
			);


			// now work out friction parts
			// direction of friction linear
			Vec3d tal = (n.cross(vab1)).cross(n);
			Vec3d tbl = (n.cross(vba1)).cross(n);

			// angular velocity impulse
			Vec3d la2f = IaInv.multiply(rap.cross((n.add(tal.normalise().scale(FRICTION))).scale(ja)));
			Vec3d lb2f = IbInv.multiply(rbp.cross((n.neg().add(tbl.normalise().scale(FRICTION))).scale(jb)));

			// now just do a hack where i transfer some angular velocity from one to other in proportion to mass, likes it linear momentum
			Vec3d angularMomentumSentA = rigidBody.getAngularVelocity().scale(ma / (ma + mb));
			Vec3d angularMomentumSentB = otherBody.getAngularVelocity().scale(mb / (ma + mb));

			Vec3d angularMomentumReceivedA = angularMomentumSentB.scale(mb * ANGULAR_MOMENTUM_LOST);
			Vec3d angularMomentumReceivedB = angularMomentumSentA.scale(ma * ANGULAR_MOMENTUM_LOST);

			// velocity impulse
			Vec3d va2 = (n.add(tal.scale(FRICTION))).scale(ja/ma);
			Vec3d vb2 = (n.neg().add(tbl.scale(FRICTION))).scale(jb/mb);

			// add impulse
			rigidBody.addImpulse(n.scale(-collisionDist/2.0), va2, la2f.add(angularMomentumReceivedA).subtract(angularMomentumSentA));
			otherBody.addImpulse(n.scale(collisionDist/2.0), vb2.neg(), lb2f.neg().add(angularMomentumReceivedB).subtract(angularMomentumSentB));

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
