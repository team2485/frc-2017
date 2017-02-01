package org.usfirst.frc.team2485.util;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

public class ScalingMax extends WarlordsControlSystem {

	private double xVal;
	private double yVal;
	private double ratio;
	private double max;
	private boolean isNegX = false;
	private boolean isNegY = false;

	public ScalingMax(PIDOutput[] outputs, PIDSource[] sources) {
		super(outputs, sources);
		// TODO Auto-generated constructor stub
	}

	@Override
	protected void calculate() {
		// TODO Auto-generated method stub

		// check if the values are even above maximum
		if (xVal > max || yVal > max) {

			// check if values are negative
			if (xVal < 0) {
				isNegX = true;
				xVal = Math.abs(xVal);
			}

			if (yVal < 0) {
				isNegY = true;
				yVal = Math.abs(yVal);
			}

			// check all possible factors (starting at the top and working down
			// to find the greatest possible factor)
			// and check to make sure the scaled down version is scaled down
			// enough

			/*
			 * QUESTION: what if it can't be scaled down whole-numberedly
			 * enough?
			 */

			for (int factor = (int) Math.ceil(max); factor > 2; factor--) {
				if (xVal % factor == 0) {
					if (yVal % factor == 0) {
						if (((yVal / factor) < max) && ((xVal / factor) < max)) {
							ratio = factor;
							break;
						}
					}
				}
			}

			// reassign negatives if needed
			if (isNegX)
				xVal = -(xVal / ratio);
			else
				xVal = (xVal / ratio);

			if (isNegY)
				yVal = -(yVal / ratio);
			else
				yVal = (yVal / ratio);

		}

	}

}
