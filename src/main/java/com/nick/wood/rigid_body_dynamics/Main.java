package com.nick.wood.rigid_body_dynamics;

import com.nick.wood.rigid_body_dynamics.graphics.Game;
import com.nick.wood.rigid_body_dynamics.graphics.Inputs;
import com.nick.wood.rigid_body_dynamics.maths.Vec3d;
import com.nick.wood.rigid_body_dynamics.particle_system_dynamics_verbose.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Random;
import java.util.UUID;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class Main {

    public static void main(String[] args) throws ExecutionException, InterruptedException {

        Inputs inputs = new Inputs();

        SimulationInterface simulation = new com.nick.wood.rigid_body_dynamics.rigid_body_dynamics_verbose.Simulation(inputs);

        Game game = new Game(1000, 800, simulation);

        ExecutorService executor = Executors.newFixedThreadPool(4);

        Future<?> submit = executor.submit(game);

        // waits for game to finish
        submit.get();

        // closes executor service
        executor.shutdown();
        


    }

    private void particleSim() throws ExecutionException, InterruptedException {
        HashMap<UUID, Particle> particles = new HashMap<>();

        Random random = new Random();

        for (int i = 0; i < 20; i+=2) {
            for (int j = 0; j < 20; j+=2) {
                ArrayList<NaryForce> localNaryForces = new ArrayList<>();
                localNaryForces.add(new ViscousDrag(random.nextInt(4) + 1));
                localNaryForces.add(new Gravity());
                Particle particle = new Particle(random.nextInt(4) + 1, new Vec3d(i, j, 20.0), Vec3d.ZERO, localNaryForces);
                particles.put(particle.getUuid(), particle);
            }
        }

        ArrayList<Plane> planes = new ArrayList<>();
        Plane ground = new Plane(new Vec3d(0.0, 0.0, -10.0), new Vec3d(0.0, 0.0, 1.0));
        planes.add(ground);

        ArrayList<NaryForce> naryForces = new ArrayList<>();

        Simulation simulation = new Simulation(particles, naryForces, planes);

        Game game = new Game(1000, 800, simulation);

        ExecutorService executor = Executors.newFixedThreadPool(4);

        Future<?> submit = executor.submit(game);

        // waits for game to finish
        submit.get();

        // closes executor service
        executor.shutdown();
    }

}
