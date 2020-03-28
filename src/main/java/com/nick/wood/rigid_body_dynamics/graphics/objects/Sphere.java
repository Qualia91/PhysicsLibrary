package com.nick.wood.rigid_body_dynamics.graphics.objects;

import com.nick.wood.rigid_body_dynamics.graphics.Mesh;
import com.nick.wood.rigid_body_dynamics.graphics.Vertex;
import com.nick.wood.rigid_body_dynamics.maths.Vec3d;

import java.util.ArrayList;

public class Sphere implements MeshObject {

	private static final int SECTOR_COUNT = 4;
	private static final int STACK_COUNT = 2;
	private static final double RADIUS = 10.0;

	private final Mesh mesh;

	public Sphere() {

		//Vertex[] vertices = new Vertex[SECTOR_COUNT * STACK_COUNT + 1];

		ArrayList<Vertex> verticesArray = new ArrayList<>();

		double sectorStep = 2 * Math.PI / SECTOR_COUNT;
		double stackStep = Math.PI / STACK_COUNT;

		for (int stackI = 0; stackI < STACK_COUNT; stackI++) {

			double stackAngle = Math.PI/2 - stackI * stackStep; // starting at pi/2 and going to -pi/2
			double xy = RADIUS * Math.cos(stackAngle); // used for x and y calc later
			double z = RADIUS * Math.sin(stackAngle);

			if (stackI == 0) {
				verticesArray.add(new Vertex(
						new Vec3d(
								xy,
								xy,
								z
						),
						Vec3d.Z));
			}

			else {
				for (int sectorI = 0; sectorI < SECTOR_COUNT; sectorI++) {

					double sectorAngle = sectorI * sectorStep;

					verticesArray.add(new Vertex(
							new Vec3d(
									xy * Math.cos(sectorAngle),
									xy * Math.sin(sectorAngle),
									z
							),
							Vec3d.Z));

				}
			}

		}

		double stackAngle = -Math.PI/2; // starting at pi/2 and going to -pi/2
		double xy = RADIUS * Math.cos(stackAngle); // used for x and y calc later
		double z = RADIUS * Math.sin(stackAngle);
		verticesArray.add(new Vertex(
				new Vec3d(
						xy,
						xy,
						z
				),
				Vec3d.Z));

		//int[] indices = new int[SECTOR_COUNT * STACK_COUNT  * 3];

		ArrayList<Integer> indicesArray = new ArrayList<>();



		Vertex[] vertices = new Vertex[verticesArray.size()];
		for (int i = 0; i < verticesArray.size(); i++) {
			vertices[i] = verticesArray.get(i);
		}

	//int[] indices = new int[indicesArray.size()];
	//for (int i = 0; i < indicesArray.size(); i++) {
	//	indices[i] = indicesArray.get(i);
	//}

		int[] indices = new int[] {
				0, 1, 2,
				0, 2, 3,
				0, 3, 4,
				0, 4, 1,
				1, 5, 2,
				2, 5, 3,
				3, 5, 4,
				4, 5, 1
		};
		mesh = new Mesh(vertices, indices);
	}

	private double calcSectorAngle(double sectorStep, double sectorCount) {
		return 2 * Math.PI * (sectorStep / sectorCount);
	}

	private double calcStackAngle(double stackStep, double stackCount) {
		return (Math.PI/2.0) - (Math.PI * (stackStep / stackCount));
	}

	public Mesh getMesh() {
		return mesh;
	}
}
