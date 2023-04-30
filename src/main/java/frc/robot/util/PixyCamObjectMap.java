// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** Add your docs here. */
public class PixyCamObjectMap {

	private PixyCamObject[] m_objectMap = new PixyCamObject[256];

	public synchronized PixyCamObject get(int index) {
		return m_objectMap[index];
	}

	public synchronized void set(int index, PixyCamObject element) {
		m_objectMap[index] = element;
	}

	public synchronized int size() {
		return m_objectMap.length;
	}
}
