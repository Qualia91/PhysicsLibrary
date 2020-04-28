package com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose;

import com.nick.wood.graphics_library.input.Inputs;
import com.nick.wood.maths.objects.Quaternion;
import com.nick.wood.maths.objects.vector.Vec3d;
import com.nick.wood.rigid_body_dynamics.SimulationInterface;
import com.nick.wood.rigid_body_dynamics.game.Game;
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.forces.Force;
import com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.forces.GravityBasic;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Random;
import java.util.UUID;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

import static org.junit.jupiter.api.Assertions.*;

class SimulationTest {

	@Test
	void simBasicTest() throws ExecutionException, InterruptedException {

		ArrayList<RigidBody> rigidBodies = new ArrayList<>();

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


		UUID uuid2 = UUID.randomUUID();
		RigidBody rigidBody2 = new RigidBody(uuid2, 1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(0.0, 0.0, 0.0), quaternion, Vec3d.X, Vec3d.ZERO, RigidBodyType.SPHERE,forces);
		rigidBodies.add(rigidBody2);

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

		// create player
		//playerRigidBodyUUID = UUID.randomUUID();
		//RigidBody playerRigidBody = new RigidBody(playerRigidBodyUUID, 0.1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(0.0, 0.0, 0), new Quaternion(1.0, 0.0, 0.0, 0.0), Vec3d.ZERO, Vec3d.Z.scale(0.0), RigidBodyType.SPHERE, forces);
		//rigidBodies.add(playerRigidBody);
		//RootGameObject playerRootObjectObject = convertToPlayerObject(rigidBody);
		//rootGameObjectHashMap.put(playerRigidBodyUUID, playerRootObjectObject);

		Inputs inputs = new Inputs();

		SimulationInterface simulation = new com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.Simulation(inputs, rigidBodies);

		Game game = new Game(1000, 800, simulation);

		ExecutorService executor = Executors.newFixedThreadPool(4);

		Future<?> submit = executor.submit(game);

		// waits for game to finish
		submit.get();

		// closes executor service
		executor.shutdown();
	}

	@Test
	void twoLinesInteracting() throws ExecutionException, InterruptedException {

		ArrayList<RigidBody> rigidBodies = new ArrayList<>();

		ArrayList<Force> forces = new ArrayList<>();
		Quaternion quaternion = Quaternion.RotationX(0.0);

		// demo 1: 2 lines interacting
		for (int j = 0; j < 10; j++) {
			for (int i = 0; i < 3; i++) {
				Vec3d mom = Vec3d.Z.scale(i + j/10.0);// * (j/10.0));
				Vec3d ang = Vec3d.X.scale(0.001).scale(j);
				//Vec3d ang = Vec3d.ZERO;
				if (i > 0) {
					mom = mom.neg();
					//ang = ang.neg();
					//ang = Vec3d.X.scale(0.01).scale(j);
					ang = Vec3d.ZERO;
				}
				UUID uuidR = UUID.randomUUID();
				RigidBody rigidBodyR = new RigidBody(uuidR, 1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(5.0, j*3.0 - 2*i/3.0, i * 8), quaternion, mom, ang, RigidBodyType.SPHERE,forces);
				rigidBodies.add(rigidBodyR);
			}
		}

		Inputs inputs = new Inputs();

		SimulationInterface simulation = new com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.Simulation(inputs, rigidBodies);

		Game game = new Game(1000, 800, simulation);

		ExecutorService executor = Executors.newFixedThreadPool(4);

		Future<?> submit = executor.submit(game);

		// waits for game to finish
		submit.get();

		// closes executor service
		executor.shutdown();
	}

	@Test
	void randomBox() throws ExecutionException, InterruptedException {

		ArrayList<RigidBody> rigidBodies = new ArrayList<>();

		ArrayList<Force> forces = new ArrayList<>();

		// demo 2: random box
		Random random = new Random();
		for (int k = 0; k < 5; k++) {
			for (int j = 0; j < 5; j++) {
				for (int i = 0; i < 5; i++) {
					UUID uuid = UUID.randomUUID();
					Vec3d mom = Vec3d.X.scale(random.nextInt(10) - 4).add(Vec3d.Y.scale(random.nextInt(10) - 4)).add(Vec3d.Z.scale(random.nextInt(10) - 4));
					Vec3d angMom = Vec3d.X.scale(random.nextInt(10) - 4).add(Vec3d.Y.scale(random.nextInt(10) - 4)).add(Vec3d.Z.scale(random.nextInt(10) - 4));
					RigidBody rigidBody = new RigidBody(uuid, 1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(j * 10, i * 10, k*10), new Quaternion(1.0, 0.0, 0.0, 0.0), mom, angMom.scale(0.02), RigidBodyType.SPHERE, forces);
					rigidBodies.add(rigidBody);
				}
			}
		}

		Inputs inputs = new Inputs();

		SimulationInterface simulation = new com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.Simulation(inputs, rigidBodies);

		Game game = new Game(1000, 800, simulation);

		ExecutorService executor = Executors.newFixedThreadPool(4);

		Future<?> submit = executor.submit(game);

		// waits for game to finish
		submit.get();

		// closes executor service
		executor.shutdown();
	}

	@Test
	void bigBang() throws ExecutionException, InterruptedException {

		ArrayList<RigidBody> rigidBodies = new ArrayList<>();

		ArrayList<Force> forces = new ArrayList<>();

		// demo 3: big bang
		Random random = new Random();
		for (int k = -2; k < 2; k++) {
			for (int j = -2; j < 2; j++) {
				for (int i = -2; i < 2; i++) {
					Vec3d mom = Vec3d.X.scale(-i).add(Vec3d.Y.scale(-j)).add(Vec3d.Z.scale(-k));
					Vec3d angMom = Vec3d.X.scale(random.nextInt(10) - 4).add(Vec3d.Y.scale(random.nextInt(10) - 4)).add(Vec3d.Z.scale(random.nextInt(10) - 4));
					UUID uuid = UUID.randomUUID();
					RigidBody rigidBody = new RigidBody(uuid, 1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(i * 10, j * 10, k*10), new Quaternion(1.0, 0.0, 0.0, 0.0), mom, angMom.scale(0.02), RigidBodyType.SPHERE, forces);
					rigidBodies.add(rigidBody);
				}
			}
		}

		Inputs inputs = new Inputs();

		SimulationInterface simulation = new com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.Simulation(inputs, rigidBodies);

		Game game = new Game(1000, 800, simulation);

		ExecutorService executor = Executors.newFixedThreadPool(4);

		Future<?> submit = executor.submit(game);

		// waits for game to finish
		submit.get();

		// closes executor service
		executor.shutdown();
	}

	@Test
	void cubeSphereInteraction() throws ExecutionException, InterruptedException {

		ArrayList<RigidBody> rigidBodies = new ArrayList<>();

		ArrayList<Force> forces = new ArrayList<>();
		forces.add(new GravityBasic());
		ArrayList<Force> forces2 = new ArrayList<>();

		UUID uuid2 = UUID.randomUUID();
		RigidBody rigidBody2 = new RigidBody(uuid2, 1, new Vec3d(1.0, 1.0, 1.0), new Vec3d(0.0, -3.0, 5.0 - 0.5), new Quaternion(1.0, 0.0, 0.0, 0.0), Vec3d.Y, Vec3d.X.scale(0), RigidBodyType.SPHERE, forces);
		rigidBodies.add(rigidBody2);

		UUID floorUUID = UUID.randomUUID();
		RigidBody floorRigidBody = new RigidBody(floorUUID, 1000000, new Vec3d(50.0, 50.0, 1.0), new Vec3d(0.0, 0.0, -1.0), Quaternion.RotationY(0.0), Vec3d.ZERO, Vec3d.ZERO, RigidBodyType.CUBOID, forces2);
		rigidBodies.add(floorRigidBody);

		Inputs inputs = new Inputs();

		SimulationInterface simulation = new com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.Simulation(inputs, rigidBodies);

		Game game = new Game(1000, 800, simulation);

		ExecutorService executor = Executors.newFixedThreadPool(4);

		Future<?> submit = executor.submit(game);

		// waits for game to finish
		submit.get();

		// closes executor service
		executor.shutdown();
	}

}