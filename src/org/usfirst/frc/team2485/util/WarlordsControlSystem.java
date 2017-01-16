package org.usfirst.frc.team2485.util;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Timer;
import java.util.TimerTask;



import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

public abstract class WarlordsControlSystem implements PIDOutput {
	
	
	protected PIDOutput[] outputs;

	private boolean enabled = false;

	private long period = DEFAULT_PERIOD;
	private static final long DEFAULT_PERIOD = 10;

	private Timer pidTimer;

	public WarlordsControlSystem(PIDOutput[] outputs) {
		this.outputs = outputs;
		pidTimer = new Timer(true);
		pidTimer.schedule(new PIDTask(), 0, period);
	}

	/**
	 * @return time between PID calculations (millis)
	 */
	public long getPeriod() {
		return period;
	}
	
	public void setPeriod(long period) {
		this.period = period;
		pidTimer.cancel();
		pidTimer = new Timer(true);
		pidTimer.schedule(new PIDTask(), 0, period);
	}
	
	/**
	 * @return true if PID is currently controlling the output motor
	 */
	public boolean isEnabled() {
		return enabled;
	} 


	public void enable() {
		this.enabled = true;
	}

	/**
	 * Disables and clears integral and derivative terms
	 */
	public void disable() {
		this.enabled = false;
		
	}

	

	/**
	 * Frees all resources related to PID calculations
	 */
	public void finalize() {
		pidTimer.cancel();
		pidTimer = null;
		outputs = null;
	}

	/**
	 * Calculates output based on sensorVal but does not read from source or write to output directly
	 */
	protected abstract void calculate();
	
	private class PIDTask extends TimerTask {

		@Override
		public void run() {
			if (enabled) {

				calculate();

			}

		}

	}
}
