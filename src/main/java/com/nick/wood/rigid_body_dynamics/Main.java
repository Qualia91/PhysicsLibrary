package com.nick.wood.rigid_body_dynamics;

import com.nick.wood.rigid_body_dynamics.graphics.Game;

import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class Main {

    public static void main(String[] args) throws ExecutionException, InterruptedException {

        Game game = new Game(1000, 800, () -> {});

        ExecutorService executor = Executors.newFixedThreadPool(4);

        Future<?> submit = executor.submit(game);

        // waits for game to finish
        submit.get();

        // closes executor service
        executor.shutdown();


    }

}
