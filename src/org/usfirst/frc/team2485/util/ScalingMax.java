package org.usfirst.frc.team2485.util;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

public class ScalingMax extends WarlordsControlSystem {

	private double[] inVals, outVals;

	public ScalingMax(PIDOutput[] outputs, PIDSource[] sources) {
		super(outputs, sources);
		if (outputs.length != sources.length) throw new IllegalArgumentException("not same size arrays");
		inVals = new double[sources.length]; 
		outVals = new double[outputs.length]; 

	}
	
	public double[] getInValues() {
		return inVals;
	}
	
	public double[] getOutValues() {
		return outVals;
	}
	
	@Override
	protected void calculate() {
		
		for (int i = 0; i < inVals.length; i++) {
			inVals[i] = sources[i].pidGet();
		}
		
		double maxRatio = 1;
		for (double x : inVals) {
			double curRatio = Math.abs(x / setpoint);
			maxRatio = Math.max(maxRatio, curRatio);
		}
			
		for (int i = 0; i < outVals.length; i++) {
			outVals[i] = inVals[i] / maxRatio;
			outputs[i].pidWrite(outVals[i]);
		}

	}

}
