package com.nick.wood.rigid_body_dynamics.game;

import com.nick.wood.graphics_library.Window;
import com.nick.wood.rigid_body_dynamics.SimulationInterface;

public class Game implements Runnable {

	private final SimulationInterface simulation;
	private double simHerts = 60;
	private final Window window;

	public Game(int width,
	            int height,
	            SimulationInterface simulation,
	            boolean enableCameraView,
	            boolean enableCameraMove) {

		this.simulation = simulation;
		this.window = new Window(
				width,
				height,
				"",
				simulation.getRootGameObjects(),
				simulation.getInputs(),
				enableCameraView, enableCameraMove);
	}

	@Override
	public void run() {

		window.init();

		long lastTime = System.nanoTime();

		double deltaSeconds = 0.0;

		while (!window.shouldClose()) {

			long now = System.nanoTime();

			deltaSeconds += (now - lastTime) / 1000000000.0;

			while (deltaSeconds >= 1 / simHerts) {

				simulation.iterate(deltaSeconds);

				window.setTitle("Iteration time: " + deltaSeconds);

				deltaSeconds = 0.0;

			}

			window.loop();

			lastTime = now;

		}

		window.destroy();

	}

}
