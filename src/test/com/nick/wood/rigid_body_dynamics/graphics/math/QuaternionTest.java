package com.nick.wood.rigid_body_dynamics.graphics.math;

import com.nick.wood.rigid_body_dynamics.maths.Matrix4d;
import com.nick.wood.rigid_body_dynamics.maths.Quaternion;
import com.nick.wood.rigid_body_dynamics.maths.Vec3d;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

class QuaternionTest {

	@Test
	void example90DegRotAroundZ() {

		Matrix4d rotationMatrix = new Matrix4d(
				0, -1, 0.0, 0.0,
				1, 0, 0.0, 0.0,
				0.0, 0.0, 1.0, 0.0,
				0.0, 0.0, 0.0, 1.0
		);

		double trace = rotationMatrix.trace();

		assertEquals(trace, 1.0, 0.000000001);

		Quaternion quaternion = rotationMatrix.toQuaternion();

		assertEquals(quaternion.getQ()[0], Math.sqrt(2)/2, 0.0000001);
		assertEquals(quaternion.getQ()[1], 0.0, 0.0000001);
		assertEquals(quaternion.getQ()[2], 0.0, 0.0000001);
		assertEquals(quaternion.getQ()[3], Math.sqrt(2)/2, 0.0000001);

		Quaternion rotationFromAngle = Quaternion.RotationZ(Math.toRadians(90));

		assertArrayEquals(rotationFromAngle.getQ(), quaternion.getQ(), 0.000001);

		Vec3d vectorToRotate = new Vec3d(1, 0, 0);

		Quaternion vector = new Quaternion(vectorToRotate);

		assertEquals(vector.getQ()[0], 0.0, 0.000001);
		assertEquals(vector.getQ()[1], 1.0, 0.000001);
		assertEquals(vector.getQ()[2], 0.0, 0.000001);
		assertEquals(vector.getQ()[3], 0.0, 0.000001);

		Quaternion rotatedVector = rotationFromAngle.rotateVector(vector);
		Vec3d rotatedVec3d = rotatedVector.toVec3d();
		Vec3d expectedRotatedVector = new Vec3d(0.0, 1.0, 0.0);

		assertArrayEquals(rotatedVec3d.getValues(), expectedRotatedVector.getValues(), 0.000001);

	}
}