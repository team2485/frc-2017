package org.usfirst.frc.team2485.util;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PIDSourceWrapper implements PIDSource {
	
	private WARLordsPIDSource pidSource;

	public void setPidSource(WARLordsPIDSource pidSource) {
		this.pidSource = pidSource;
	}
	
	@Override
	public PIDSourceType getPIDSourceType() {
		return null;
	}

	@Override
	public double pidGet() {
		return pidSource.pidGet();
	}

	@Override
	public void setPIDSourceType(PIDSourceType arg0) {
		
	}
	
}
