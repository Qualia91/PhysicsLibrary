package com.boc_dev.physics_library;

import com.boc_dev.maths.objects.QuaternionD;
import com.boc_dev.maths.objects.vector.Vecd;

import java.util.UUID;

public interface Body {
	UUID getUuid();

	Vecd getOrigin();

	QuaternionD getRotation();
}
