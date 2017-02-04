package org.usfirst.frc.team2485.util;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PIDSourceWrapper implements PIDSource {
	public PIDSourceWrapper(WARLordsPIDSource pidSource) {
		// TODO Auto-generated constructor stub
		this.pidSource = pidSource;
	}
	private WARLordsPIDSource pidSource;

	@Override
	public PIDSourceType getPIDSourceType() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public double pidGet() {
		// TODO Auto-generated method stub
		return pidSource.pidGet();
	}

	@Override
	public void setPIDSourceType(PIDSourceType arg0) {
		// TODO Auto-generated method stub
		
	}
	
}
