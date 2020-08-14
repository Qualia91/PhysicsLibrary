package com.nick.wood.physics_library;

import com.nick.wood.physics_library.rigid_body_dynamics_verbose.RigidBody;

import java.util.ArrayList;

public interface SimulationInterface {

	void iterate(double deltaSeconds, ArrayList<RigidBody> rigidBodies);

	ArrayList<? extends Body> getBodies();
}
