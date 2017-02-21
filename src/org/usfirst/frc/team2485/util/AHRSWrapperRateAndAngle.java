package org.usfirst.frc.team2485.util;

import org.usfirst.frc.team2485.robot.RobotMap;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class AHRSWrapperRateAndAngle implements PIDSource {
	public static enum Units {
		DEGS, RADS
	}
	private PIDSourceType pidSource;
	private double rate;
	private boolean run = true;
	private Units units;
	public AHRSWrapperRateAndAngle(PIDSourceType pidSource, Units units) {
		this.pidSource = pidSource;
		this.units = units;
		new RateThread().start();
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
			return (units == Units.RADS) ? Math.PI / 180 * rate : rate;
		}
	}
	
	@Override
	protected void finalize() throws Throwable {
		run = false;
	}
	
	private class RateThread extends Thread {
		@Override
		public void run() {
			double last = RobotMap.ahrs.getAngle();
			while (run) {
				double cur = RobotMap.ahrs.getAngle();
				double diff = cur - last;
				if (diff < -180) {
					diff += 360;
				} else if (diff > 180) {
					diff -= 360;
				}
				last = cur;
				rate = diff * 100;
				try {
					Thread.sleep(10);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}
	}
}
