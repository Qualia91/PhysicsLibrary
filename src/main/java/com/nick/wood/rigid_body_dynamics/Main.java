package com.nick.wood.rigid_body_dynamics;

import com.nick.wood.rigid_body_dynamics.graphics.Game;
import com.nick.wood.rigid_body_dynamics.graphics.math.Vec3d;
import com.nick.wood.rigid_body_dynamics.particle_system_dynamics_verbose.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.UUID;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class Main {

    public static void main(String[] args) throws ExecutionException, InterruptedException {

        HashMap<UUID, Particle> particles = new HashMap<>();
        Particle particle = new Particle(1, Vec3d.ZERO, Vec3d.ZERO, new ArrayList<>());
        particles.put(particle.getUuid(), particle);

        ArrayList<NaryForce> naryForces = new ArrayList<>();
        naryForces.add(new Gravity());
        naryForces.add(new ViscousDrag(1));

        Simulation simulation = new Simulation(particles, naryForces);

        Game game = new Game(1000, 800, simulation);

        ExecutorService executor = Executors.newFixedThreadPool(4);

        Future<?> submit = executor.submit(game);

        // waits for game to finish
        submit.get();

        // closes executor service
        executor.shutdown();


    }

}
