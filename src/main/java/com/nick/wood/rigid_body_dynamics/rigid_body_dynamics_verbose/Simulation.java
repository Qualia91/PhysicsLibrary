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
import com.nick.wood.graphics_library.objects.mesh_objects.SphereMesh;
import com.nick.wood.maths.objects.matrix.Matrix4f;
import com.nick.wood.maths.objects.vector.Vec3f;
import com.nick.wood.rigid_body_dynamics.SimulationInterface;
import com.nick.wood.rigid_body_dynamics.game.controls.FlightAssistControl;
import com.nick.wood.graphics_library.input.*;
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

	public Simulation(Inputs input, ArrayList<RigidBody> rigidBodies) {

		this.rigidBodies = rigidBodies;
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

		// todo loop through rigid bodies and convert to game objects
		for (RigidBody rigidBody : rigidBodies) {
			rootGameObjectHashMap.put(rigidBody.getUuid(), convertToGameObject(rigidBody));
		}

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
				10);

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
