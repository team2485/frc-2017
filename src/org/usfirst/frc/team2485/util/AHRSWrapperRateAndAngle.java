package org.usfirst.frc.team2485.util;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class AHRSWrapperRateAndAngle implements PIDSource {
	public static enum Units {
		DEGS, RADS
	}
	private PIDSourceType pidSource;
	private Units units;
	public AHRSWrapperRateAndAngle(PIDSourceType pidSource, Units units) {
		this.pidSource = pidSource;
		this.units = units;
	}
	
	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		this.pidSource = pidSource;
	}
	
	@Override
	public PIDSourceType getPIDSourceType() {
		return pidSource;
	}
	
	@Override
	public double pidGet() {
		if (pidSource == PIDSourceType.kDisplacement) {
			return (units == Units.RADS) ? Math.PI / 180 * RobotMap.ahrs.getAngle() : RobotMap.ahrs.getAngle();
		} else {
			double rate = RobotMap.ahrs.getRate() * 60; // multiply by 60 because kauai labs can't math
			if (Math.abs(rate) > 360) {
				rate = 0;
			}
			return (units == Units.RADS) ? Math.PI / 180 * rate : rate;
		}
	}
	
	
}
