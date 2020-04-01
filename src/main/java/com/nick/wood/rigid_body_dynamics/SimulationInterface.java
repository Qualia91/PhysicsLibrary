package com.nick.wood.rigid_body_dynamics;

import com.nick.wood.rigid_body_dynamics.game.controls.Inputs;
import com.nick.wood.rigid_body_dynamics.game.game_objects.GameObject;
import com.nick.wood.rigid_body_dynamics.particle_system_dynamics_verbose.Plane;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.UUID;

public interface SimulationInterface {

	HashMap<UUID, GameObject> getGameObjects();

	void iterate(double deltaSeconds);

	ArrayList<Plane> getPlanes();

	Inputs getInputs();
}
