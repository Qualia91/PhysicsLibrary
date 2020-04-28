package com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose;

import com.nick.wood.graphics_library.Material;
import com.nick.wood.graphics_library.lighting.DirectionalLight;
import com.nick.wood.graphics_library.lighting.PointLight;
import com.nick.wood.graphics_library.lighting.SpotLight;
import com.nick.wood.graphics_library.objects.Camera;
import com.nick.wood.graphics_library.objects.Transform;
import com.nick.wood.graphics_library.objects.game_objects.*;
import com.nick.wood.graphics_library.objects.mesh_objects.CubeMesh;
import com.nick.wood.graphics_library.objects.mesh_objects.MeshObject;
import com.nick.wood.graphics_library.objects.mesh_objects.ModelMesh;
import com.nick.wood.graphics_library.objects.mesh_objects.SphereMesh;
import com.nick.wood.maths.objects.matrix.Matrix4f;
import com.nick.wood.maths.objects.vector.Vec3f;
import com.nick.wood.rigid_body_dynamics.SimulationInterface;
import com.nick.wood.rigid_body_dynamics.game.controls.FlightAssistControl;
import com.nick.wood.graphics_library.input.*;
import com.nick.wood.maths.objects.matrix.Matrix4d;
import com.nick.wood.maths.objects.Quaternion;
import com.nick.wood.maths.objects.vector.Vec3d;
import com.nick.wood.rigid_body_dynamics.particle_system_dynamics_verbose.Plane;
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.forces.Force;
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.ode.RigidBodyODEReturnData;
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.ode.RungeKutta;

import java.util.*;

public class Simulation implements SimulationInterface {

	private final RungeKutta rungeKutta;
	private final Inputs input;
	private final CollisionDetection collisionDetection;
	private final Game3DInputs game3DInputs;
	private UUID playerRigidBodyUUID;
	private final Control control;

	ArrayList<RigidBody> rigidBodies = new ArrayList<>();
	HashMap<UUID, RootGameObject> rootGameObjectHashMap = new HashMap<>();
	ArrayList<Plane> planes = new ArrayList<>();

	public Simulation(Inputs input) {

		this.input = input;

		this.control = new FlightAssistControl(0.005, 5.0);
		this.game3DInputs = new Game3DInputs(input, control);

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

		//ArrayList<Force> forces2 = new ArrayList<>();
		//UUID uuid = UUID.randomUUID();
		//RigidBody rigidBody = new RigidBody(uuid, 1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(2.0, 0.0, 0.0), new Quaternion(1.0, 0.0, 0.0, 0.0), Vec3d.X.scale(10), Vec3d.Y.scale(0.1), RigidBodyType.SPHERE,forces);
		//rigidBodies.add(rigidBody);
		//uuidGameObjectHashMap.put(uuid, convertToGameObject(rigidBody, 10));

		//// Arena
		//UUID uuidArena = UUID.randomUUID();
		//RigidBody rigidBodyArena = new RigidBody(uuidArena, 10000, new Vec3d(100, 100, 100), new Vec3d(0.0, 0.0, 0.0), new Quaternion(1.0, 0.0, 0.0, 0.0), Vec3d.ZERO, Vec3d.ZERO, RigidBodyType.SPHERE_INNER, forces);
		//rigidBodies.add(rigidBodyArena);
		//uuidGameObjectHashMap.put(uuidArena, convertToGameObject(rigidBodyArena, 10));

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
		UUID uuid = UUID.randomUUID();
		Quaternion quaternion = Quaternion.RotationX(0.0);
		RigidBody rigidBody = new RigidBody(uuid, 1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(10.0, 0.0, 0.0), quaternion, Vec3d.ZERO, Vec3d.ZERO, RigidBodyType.SPHERE,forces);
		rigidBodies.add(rigidBody);
		rootGameObjectHashMap.put(uuid, convertToGameObject(rigidBody));


		UUID uuid2 = UUID.randomUUID();
		RigidBody rigidBody2 = new RigidBody(uuid2, 1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(0.0, 0.0, 0.0), quaternion, Vec3d.X, Vec3d.ZERO, RigidBodyType.SPHERE,forces);
		rigidBodies.add(rigidBody2);
		rootGameObjectHashMap.put(uuid2, convertToGameObject(rigidBody2));

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
		//playerRigidBodyUUID = UUID.randomUUID();
		//RigidBody playerRigidBody = new RigidBody(playerRigidBodyUUID, 0.1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(0.0, 0.0, 0), new Quaternion(1.0, 0.0, 0.0, 0.0), Vec3d.ZERO, Vec3d.Z.scale(0.0), RigidBodyType.SPHERE, forces);
		//rigidBodies.add(playerRigidBody);
		//RootGameObject playerRootObjectObject = convertToPlayerObject(rigidBody);
		//rootGameObjectHashMap.put(playerRigidBodyUUID, playerRootObjectObject);

		RootGameObject cameraRootObject = new RootGameObject();
		Camera camera = new Camera(new Vec3f(0.0f, 0.0f, 10.0f), new Vec3f(0.0f, 0.0f, 0.0f), 0.5f, 0.1f);
		CameraGameObject cameraGameObject = new CameraGameObject(cameraRootObject, camera, CameraType.PRIMARY);
		rootGameObjectHashMap.put(UUID.randomUUID(), cameraRootObject);

		RootGameObject lightRootObject = new RootGameObject();
		createLights(lightRootObject);
		rootGameObjectHashMap.put(UUID.randomUUID(), lightRootObject);


		this.collisionDetection = new CollisionDetection();
	}

	private void createLights(RootGameObject rootGameObject) {
		MeshObject meshGroupLight = new SphereMesh(10, new Material("/textures/white.png"), true);

		PointLight light = new PointLight(
				new Vec3f(0.0f, 1.0f, 0.0f),
				5000f);

		Transform lightGameObjectTransform = new Transform(
				Vec3f.X.scale(-10),
				Vec3f.ONE,
				Matrix4f.Identity
		);
		TransformGameObject transformGameObject = new TransformGameObject(rootGameObject, lightGameObjectTransform);
		LightGameObject lightGameObject = new LightGameObject(transformGameObject, light);
		MeshGameObject meshGameObject = new MeshGameObject(
				transformGameObject,
				meshGroupLight
		);
	}

	private RootGameObject convertToPlayerObject(RigidBody rigidBody) {
		RootGameObject rootObject = new RootGameObject();

		Transform transform = new Transform(
				(Vec3f) rigidBody.getOrigin().toVecf(),
				(Vec3f) rigidBody.getDimensions().toVecf(),
				rigidBody.getRotation().toMatrix().toMatrix4f()
		);

		TransformGameObject transformGameObject = new TransformGameObject(rootObject, transform);

		PlayerGameObject playerGameObject = new PlayerGameObject(transformGameObject);


		Camera camera = new Camera(new Vec3f(0.0f, 0.0f, 0.0f), new Vec3f(0.0f, 0.0f, 0.0f), 0.5f, 0.1f);

		CameraGameObject cameraGameObject = new CameraGameObject(transformGameObject, camera, CameraType.PRIMARY);

		MeshObject meshObject;

		switch (rigidBody.getType()){
			case SPHERE_INNER:
				meshObject = new SphereMesh(10, new Material("/textures/white.png"), true);
				break;
			case SPHERE:
				meshObject = new SphereMesh(10, new Material("/textures/white.png"), false);
				break;
			case CUBOID:
				meshObject = new CubeMesh(false, new Material("/textures/white.png")) ;
				break;
			default:
				meshObject = new SphereMesh(10, new Material("/textures/white.png"), false);
				break;
		}

		MeshGameObject meshGameObject = new MeshGameObject(
				transformGameObject,
				meshObject
		);

		return rootObject;
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

	@Override
	public HashMap<UUID, RootGameObject> getRootGameObjects() {
		return rootGameObjectHashMap;
	}

	public RootGameObject convertToGameObject(RigidBody rigidBody) {

		RootGameObject rootObject = new RootGameObject();

		Transform transform = new Transform(
				(Vec3f) rigidBody.getOrigin().toVecf(),
				(Vec3f) rigidBody.getDimensions().toVecf(),
				rigidBody.getRotation().toMatrix().toMatrix4f()
		);

		TransformGameObject transformGameObject = new TransformGameObject(rootObject, transform);

		MeshObject meshObject;
		Matrix4f startingTranslation = Matrix4f.Translation(Vec3f.X).multiply(Matrix4f.Rotation(-90f, Vec3f.X));

		switch (rigidBody.getType()){
			case SPHERE_INNER:
				meshObject = new SphereMesh(10, new Material("/textures/white.png"), true);
				break;
			case SPHERE:
				meshObject = new SphereMesh(10, new Material("/textures/white.png"), false);
				break;
			case CUBOID:
				meshObject = new CubeMesh(false, new Material("/textures/white.png"));
				break;
			default:
				meshObject = new SphereMesh(10, new Material("/textures/white.png"), false);
				break;
		}

		MeshGameObject meshGameObject = new MeshGameObject(
				transformGameObject,
				meshObject
		);

		return rootObject;

	}

	@Override
	public void iterate(double deltaSeconds) {

		// user controls
		ArrayList<RigidBody> tempList = new ArrayList<>();

		for (RigidBody rigidBody : rigidBodies) {

			if (rigidBody.getUuid().equals(playerRigidBodyUUID)) {
				rigidBody.setMomentums(control.getLinearMomentum(rigidBody.getRotation().toMatrix(), rigidBody.getLinearMomentum()), control.getAngularMomentum(rigidBody.getRotation().toMatrix(), rigidBody.getAngularMomentum()));
				control.reset();
			}
			tempList.add(rungeKutta.solve(rigidBody, rigidBody.getUuid(), deltaSeconds));
		}

		rigidBodies = tempList;

		collisionDetection.collisionDetection(tempList);

		game3DInputs.checkInputs();

		for (RigidBody rigidBody : tempList) {
			rigidBody.applyImpulse();
		}

		mapToGameObjects(rootGameObjectHashMap, rigidBodies);

	}

	private static void mapToGameObjects(HashMap<UUID, RootGameObject> gameObjects, ArrayList<RigidBody> rigidBodies) {

		for (RigidBody rigidBody : rigidBodies) {

			TransformGameObject transformGameObject = (TransformGameObject) gameObjects.get(rigidBody.getUuid()).getGameObjectNodeData().getChildren().get(0);
			transformGameObject.setPosition((Vec3f) rigidBody.getOrigin().toVecf());
			transformGameObject.setRotation(rigidBody.getRotation().toMatrix().toMatrix4f());

		}

	}

	@Override
	public ArrayList<Plane> getPlanes() {
		return planes;
	}
}
