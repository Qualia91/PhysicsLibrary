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

		assertEquals(quaternion.getS(), Math.sqrt(2)/2, 0.0000001);
		assertEquals(quaternion.getI(), 0.0, 0.0000001);
		assertEquals(quaternion.getJ(), 0.0, 0.0000001);
		assertEquals(quaternion.getK(), Math.sqrt(2)/2, 0.0000001);

		Quaternion rotationFromAngle = Quaternion.RotationZ(Math.toRadians(90));

		assertEquals(rotationFromAngle.getS(), quaternion.getS(), 0.0000001);
		assertEquals(rotationFromAngle.getI(), quaternion.getI(), 0.0000001);
		assertEquals(rotationFromAngle.getJ(), quaternion.getJ(), 0.0000001);
		assertEquals(rotationFromAngle.getK(), quaternion.getK(), 0.0000001);

		Vec3d vectorToRotate = new Vec3d(1, 0, 0);

		Quaternion vector = new Quaternion(vectorToRotate);

		assertEquals(vector.getS(), 0.0, 0.000001);
		assertEquals(vector.getI(), 1.0, 0.000001);
		assertEquals(vector.getJ(), 0.0, 0.000001);
		assertEquals(vector.getJ(), 0.0, 0.000001);

		Quaternion rotatedVector = rotationFromAngle.rotateVector(vector);
		Vec3d rotatedVec3d = rotatedVector.toVec3d();
		Vec3d expectedRotatedVector = new Vec3d(0.0, 1.0, 0.0);

		assertArrayEquals(rotatedVec3d.getValues(), expectedRotatedVector.getValues(), 0.000001);

	}
}