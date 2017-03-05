package org.usfirst.frc.team2485.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

/**
 * @author Ben Dorsey
 */

public class BrokenEncoderWrapper implements PIDSource {
	private Encoder enc;
	private double last = 0;
	public BrokenEncoderWrapper(Encoder enc) {
		this.enc = enc;
		new MonitorThread().start();
	}
	
	@Override
	public double pidGet() {
		return last;
	}
	
	private class MonitorThread extends Thread {
		@Override
		public void run() {
			while (true) {
				double now = enc.getRate();
				if (Math.abs(last - now) < 0.005) {
					last = now;
				} else {
					last -= 0.005 * Math.abs(last - now) / (last - now);
				}
				try {
					Thread.sleep(1);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
				
			}
		}
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return null;
	}
}
