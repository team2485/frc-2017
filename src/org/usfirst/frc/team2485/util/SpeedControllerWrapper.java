package org.usfirst.frc.team2485.util;

import java.util.Arrays;

import edu.wpi.first.wpilibj.SpeedController;

/**
 * 
 * Used to act on multiple speed controllers at once, or to treat many speed
 * controllers as one. Also has the ability to ramp voltage and scale inputs.
 * 
 * @author Ben Clark
 * @author Patrick Wamsley
 * @author Anoushka Bose
 * @author Jeremy McCulloch
 */
public class SpeedControllerWrapper implements SpeedController {

	private SpeedController[] speedControllerList;
	private double[] scaleFactors;
	private RampRate rampRate;
	private double desiredPWM;
	private long period = 10;

	public SpeedControllerWrapper(SpeedController[] speedControllerList, double[] scaleFactors) {

		this.speedControllerList = speedControllerList;
		
		setScaleFactors(scaleFactors);
		
		new RampThread().start();
		
	}

	public SpeedControllerWrapper(SpeedController[] speedControllerList) {
		this(speedControllerList, null);
	}

	public SpeedControllerWrapper(SpeedController speedController) {
		this(new SpeedController[] {speedController});
	}
	
	/**
	 * Set scale factors for each motor, controlling how much each motor contributes to the overall effort <br>
	 * Values given are relative and immediately normalized
	 * 
	 * @param scaleFactors An array of non-negative values, whose proportion to each other determine the work each motor does
	 */

	public void setScaleFactors(double[] scaleFactors) {
		
		if (scaleFactors == null) {
			scaleFactors = new double[speedControllerList.length];
			Arrays.fill(scaleFactors, 1.0);
		}

		if (speedControllerList.length != scaleFactors.length) {
			System.err.println("The array of speed controllers and scaleFactors must be the same length");
		}

		double maxValue = -1;

		for (double curValue : scaleFactors) {

			if (curValue < 0) {
				System.err.println(
						"A Speed Controllers scale factor cannot be negative\nTo invert a speed controller, use it's \"setInverted\" method");
			} else {
				maxValue = Math.max(maxValue, curValue);
			}
		}

		for (int i = 0; i < scaleFactors.length; i++) {

			scaleFactors[i] /= maxValue;

		}

		this.scaleFactors = scaleFactors;
	}

	@Override
	public void pidWrite(double output) {
		set(output);
	}

	@Override
	public double get() {

		double sum = 0;

		for (SpeedController s : speedControllerList) {
			sum += s.get();
		}

		return sum / speedControllerList.length;
	}

	@Override
	public void set(double pwm) {
		if (rampRate == null) {
			setRaw(pwm);
		} else {
			desiredPWM = pwm;
		}		
	}
	
	private void setRaw(double pwm) {
		for (int i = 0; i < speedControllerList.length; i++) {
			speedControllerList[i].set(pwm * scaleFactors[i]);	
		}
	}

	/**
	 * Allow you to override the ramp rate when setting values
	 * @param pwm new pwm to take effect immediately without ramp
	 */
	public void emergencySet(double pwm) {
		setRaw(pwm);
		
		if (rampRate != null) {
			rampRate.setLastValue(pwm);
		}
	}
	
	/**
	 * Allow you to override the ramp rate when stopping motor 
	 */
	public void emergencyStop() {
		emergencySet(0);
	}

	@Override
	public void setInverted(boolean isInverted) {
		for (SpeedController s : speedControllerList) {
			s.setInverted(isInverted);
		}
	}

	@Override
	public boolean getInverted() {
		return speedControllerList[0].getInverted();
	}

	@Override
	public void disable() {
		for (SpeedController controller : speedControllerList) {
			controller.disable();
		}
	}

	/**
	 * Set maximum rate of change of voltage
	 * @param rampRate maximum rate of change per 10 ms
	 */
	public void setRampRate(double rampRate) {
		setRampRate(rampRate, rampRate);
	}
	
	/**
	 * Set maximum rate of change of voltage in each direction
	 * @param rampRateUp maximum increase in voltage per 10 ms
	 * @param rampRateDown maximum decrease in voltage per 10 ms
	 */
	public void setRampRate(double rampRateUp, double rampRateDown) {
		setRampRate(rampRateUp, rampRateDown, 10);
	}
	
	/**
	 * Set maximum rate of change of voltage in each direction
	 * @param rampRateUp maximum increase in voltage per period
	 * @param rampRateDown maximum decrease in voltage per period
	 * @param period ms, time between voltage ramp calculations 
	 */
	public void setRampRate(double rampRateUp, double rampRateDown, long period) {
		this.period = period;		
		rampRate = new RampRate(rampRateUp, rampRateDown);

	}
	
	/**
	 * Disables all voltage ramping, but does not change motor value
	 */
	public void disableVoltageRamp() {
		rampRate = null;
	}

	@Override
	public void stopMotor() {
		for (SpeedController sc : speedControllerList) {
			sc.stopMotor();
		}
	}
	
	private class RampThread extends Thread {
		@Override
		public void run() {
			super.run();
			while (true) {
				if (rampRate != null) {
					setRaw(rampRate.getNextValue(desiredPWM));
				}
				try {
					Thread.sleep(period);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}
	}
}
