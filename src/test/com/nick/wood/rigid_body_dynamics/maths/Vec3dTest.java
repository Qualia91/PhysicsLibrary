package com.nick.wood.rigid_body_dynamics.maths;

import org.junit.jupiter.api.Test;
import org.lwjgl.system.CallbackI;

import static org.junit.jupiter.api.Assertions.*;

class Vec3dTest {

	@Test
	void getX() {
		Vec3d one = new Vec3d(2, 3, 4);
		assertEquals(one.getX(), 2);
	}

	@Test
	void getY() {
		Vec3d one = new Vec3d(2, 3, 4);
		assertEquals(one.getY(), 3);
	}

	@Test
	void getZ() {
		Vec3d one = new Vec3d(2, 3, 4);
		assertEquals(one.getZ(), 4);
	}

	@Test
	void add() {
		Vec3d one = new Vec3d(2, 3, 4);
		Vec3d two = new Vec3d(5, 6, 7);
		Vec3d three = one.add(two);
		assertEquals(three.getX(), 7);
		assertEquals(three.getY(), 9);
		assertEquals(three.getZ(), 11);
	}

	@Test
	void subtract() {
		Vec3d one = new Vec3d(2, 3, 4);
		Vec3d two = new Vec3d(5, 6, 7);
		Vec3d three = one.subtract(two);
		assertEquals(three.getX(), -3);
		assertEquals(three.getY(), -3);
		assertEquals(three.getZ(), -3);
	}

	@Test
	void scale() {
		Vec3d one = new Vec3d(2, 3, 4);
		Vec3d two = one.scale(3);
		assertEquals(two.getX(), 6);
		assertEquals(two.getY(), 9);
		assertEquals(two.getZ(), 12);
	}

	@Test
	void dot() {
		Vec3d one = new Vec3d(9, 2, 7);
		Vec3d two = new Vec3d(4, 8, 10);
		double a = one.dot(two);
		assertEquals(a, 122, 0.0000000001);
	}

	@Test
	void length2() {
		Vec3d one = new Vec3d(1, 1, 1);
		assertEquals(one.length2(), 3, 0.0000000001);
	}

	@Test
	void length() {
		Vec3d one = new Vec3d(1, 1, 1);
		assertEquals(one.length(), Math.sqrt(3), 0.0000000001);
	}

	@Test
	void normalise() {
		Vec3d one = new Vec3d(3, 1, 2);
		Vec3d two = one.normalise();
		assertEquals(two.getX(), 0.802, 0.001);
		assertEquals(two.getY(), 0.267, 0.001);
		assertEquals(two.getZ(), 0.534, 0.001);
	}

	@Test
	void getValues() {
	}

	@Test
	void outerProduct() {
		Vec3d one = new Vec3d(-2, 16, 4);
		Vec3d two = new Vec3d(13, -3, -1);
		Matrix4d three = one.outerProduct(two);

		assertEquals(three.get(0, 0), -26);
		assertEquals(three.get(1, 0), 6);
		assertEquals(three.get(2, 0), 2);

		assertEquals(three.get(0, 1), 208);
		assertEquals(three.get(1, 1), -48);
		assertEquals(three.get(2, 1), -16);

		assertEquals(three.get(0, 2), 52);
		assertEquals(three.get(1, 2), -12);
		assertEquals(three.get(2, 2), -4);

	}

	@Test
	void cross() {
		Vec3d one = new Vec3d(2, 3, 4);
		Vec3d two = new Vec3d(5, 6, 7);
		Vec3d three = one.cross(two);
		assertEquals(three.getX(), -3);
		assertEquals(three.getY(), 6);
		assertEquals(three.getZ(), -3);
	}

	@Test
	void neg() {
		Vec3d one = new Vec3d(2, 3, -4);
		Vec3d three = one.neg();
		assertEquals(three.getX(), -2);
		assertEquals(three.getY(), -3);
		assertEquals(three.getZ(), 4);
	}

	@Test
	void star() {

	}

	@Test
	void multiply() {
	}

	@Test
	void min() {
	}

	@Test
	void max() {
	}
}