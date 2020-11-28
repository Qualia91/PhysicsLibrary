package com.boc_dev.physics_library.rigid_body_dynamics_verbose.forces;

public class Pair<T, U> {

	private final T key;
	private final T value;

	public Pair(T key, T value) {
		this.key = key;
		this.value = value;
	}

	public T getKey() {
		return key;
	}

	public T getValue() {
		return value;
	}
}
