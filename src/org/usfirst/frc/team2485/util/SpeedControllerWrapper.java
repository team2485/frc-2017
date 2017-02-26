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

	public SpeedControllerWrapper(SpeedController[] speedControllerList, double[] scaleFactors) {

		this.speedControllerList = speedControllerList;
		
		setScaleFactors(scaleFactors);		
	}

	public SpeedControllerWrapper(SpeedController... speedControllerList) {
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
	
	public SpeedController getController(int index) {
		return speedControllerList[index];
	}

	@Override
	public void set(double pwm) {
		for (int i = 0; i < speedControllerList.length; i++) {
			speedControllerList[i].set(pwm * scaleFactors[i]);	
		}
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
	


	@Override
	public void stopMotor() {
		for (SpeedController sc : speedControllerList) {
			sc.stopMotor();
		}
	}

}
