package com.nick.wood.physics_library;

import java.util.ArrayList;

public interface SimulationInterface {
	void iterate(double deltaSeconds);
	ArrayList<? extends Body> getBodies();
}
