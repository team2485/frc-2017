package org.usfirst.frc.team2485.util;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Timer;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;

/**
 * Class that implements a standard PID Control Loop without the abnormalities of WPI's PIDController class
 * @author Jeremy McCulloch
 */
public class WarlordsPIDController extends WarlordsControlSystem {
	
	private double kP, kI, kD, kF, kC;
	private PIDSource source;
	private PIDOutput[] outputs;
	
	private double integralTerm, lastPropTerm;
	private double setpoint;
	private double sensorVal, result;
	private boolean saturateHighFlag, saturateLowFlag;
	private double saturateError;
	private double decoupleInput;
	
	private boolean enabled = false;
	
	private double minOutput = -1, maxOutput = 1;
	
	private double percentTolerance = 0.0, absoluteTolerance = 0.0;
	private boolean usesPercentTolerance = false;
	
	private Queue<Double> errorBuffer;
	private int bufferLength;
	private static final int DEFAULT_BUFFER_LENGTH = 1;
	
	private double minInput, maxInput;
	private boolean continuous;
	
	private long period;
	private static final long DEFAULT_PERIOD = 10;
	
	private Timer pidTimer;
	
	/**
	 * 
	 * @param kP proportional term, multiplied by the current error
	 * @param kI integral term, multiplied by the total (sum) error
	 * @param kD derivative term, multiplied by the change of the error
	 * @param kF feedforward term, multiplied by the setpoint, (usually) only used in rate control
	 * @param kC coupling constant
	 * @param source input device / sensor used to monitor progress towards setpoint
	 * @param outputs output device(s) / motor(s)  used to approach setpoint
	 * @param period how often PID calculation is done (millis)
	 * @param bufferLength number of values used to calculate averageError
	 */
	public WarlordsPIDController(PIDSource source, PIDOutput... outputs) {
		super(outputs);
		this.source = source;
	
		this.errorBuffer = new LinkedList<Double>();
	}
	

		
	/**
	 * Sets the gains
	 * @param kP proportional term, multiplied by the current error
	 * @param kI integral term, multiplied by the total (sum) error
	 * @param kD derivative term, multiplied by the change of the error
	 * @param kF feedforward term, multiplied by the setpoint, (usually) only used in rate control 
	 */
	public void setPID(double kP, double kI, double kD, double kF) {
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kF = kF;
	}
	
	/**
	 * Sets the gains
	 * @param kP proportional term, multiplied by the current error
	 * @param kI integral term, multiplied by the total (sum) error
	 * @param kD derivative term, multiplied by the change of the error
	 * @param kF feedforward term, multiplied by the setpoint, (usually) only used in rate control 
	 */
	public void setPID(double kP, double kI, double kD) {
		setPID(kP, kI, kD, 0);
	} 
	
	/**
	 * @return time between PID calculations (millis)
	 */
	public long getPeriod() {
		return period;
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
		this.integralTerm = 0;
		this.lastPropTerm = 0;
	}
	
	/**
	 * Sets range for the output values calculated by the PID Controller
	 * @param minOutput minimum value to set the output motor to 
	 * @param maxOutput maximum value to set the output motor to 
	 */
	public void setOutputRange(double minOutput, double maxOutput) {
		this.minOutput = minOutput;
		this.maxOutput = maxOutput;
	}
	
	/**
	 * Sets input range to be used in continuous mode
	 * @param minInput minimum value of sensor and setpoint
	 * @param maxInput maximum value of sensor and setpoint
	 */
	public void setInputRange(double minInput, double maxInput) {
		this.minInput = minInput;
		this.maxInput = maxInput;
	}
	
	/**
	 * Sets whether sensor loops from minInput to maxInput
	 * @param continuous true if loops
	 */
	public void setContinuous(boolean continuous) {
		this.continuous = continuous;
	}
	
	/**
	 * Returns whether sensor loops from minInput to maxInput
	 * @return true if loops
	 */
	public boolean isContinuous() {
		return continuous;
	}
	
	/**
	 * Sets setpoint and clears history of errors
	 * @param setpoint new setpoint 
	 */
	public void setSetpoint(double setpoint) {
		this.setpoint = setpoint;
		synchronized (errorBuffer) {
			errorBuffer = new LinkedList<Double>();
		}
	}
	
	public double getSetpoint() {
		return setpoint;
	}
	
	/**
	 * @return error as calculated by the PID control loop
	 */
	public double getError() {
		return setpoint - source.pidGet();
	}
	
	/**
	 * @param bufferLength number of values used to calculate averageError
	 */
	public void setBufferLength(int bufferLength) {
		this.bufferLength = bufferLength;
	}
	
	/**
	 * @return number of values used to calculate averageError
	 */
	public double getBufferLength() {
		return bufferLength;
	}
	
	/**
	 * @return average of the last bufferlength errors, or fewer if not enough are available
	 */
	public double getAvgError() {
		synchronized (errorBuffer) {
			double sum = 0;
			for (Iterator<Double> iterator = errorBuffer.iterator(); iterator.hasNext();) {
				sum += (double) iterator.next();
			}
			return sum / errorBuffer.size();
		}
	}
	
	/**
	 * @param tolerance considered on target when within tolerance of setpoint
	 */
	public void setAbsoluteTolerance(double tolerance) {
		this.absoluteTolerance = tolerance; 
		usesPercentTolerance = false;
	}
	
	/**
	 * @param tolerance considered on target when within tolerance*setpoint of setpoint
	 */
	public void setPercentTolerance(double tolerance) {
		this.percentTolerance = tolerance;
		usesPercentTolerance = true;
	}
	
	/**
	 * Compares the average error to the specified tolerance
	 * @return true if within specified tolerance of setpoint
	 */
	public boolean isOnTarget() {
		if (usesPercentTolerance) {
			return Math.abs(getAvgError()) < setpoint * percentTolerance;
		} else {
			return Math.abs(getAvgError()) < absoluteTolerance;
		}
	}
	
	/**
	 * Frees all resources related to PID calculations
	 */
	public void finalize() {
		pidTimer.cancel();
		pidTimer = null;
		source = null;
		outputs = null;
	}
	
	/**
	 * Calculates output based on sensorVal but does not read from source or write to output directly
	 */
	protected synchronized void calculate() {
		
		sensorVal = source.pidGet();
		double error = setpoint - sensorVal;

		
		while (continuous && Math.abs(error) > (maxInput - minInput) / 2) {
			if (error > 0) {
				error -= maxInput - minInput;
			} else {
				error += maxInput - minInput;
			}
			
		}
		
		double propTerm = kP * error;
		double integralError = kI * propTerm + decoupleInput + kC * saturateError;
		
		if (saturateLowFlag) {
			if (integralError > 0) {
				integralTerm += integralError;
			}
		} else if (saturateHighFlag) {
			if (integralError < 0) {
				integralTerm += integralError;
			}
		} else {
			integralTerm += integralError;			
		}
		
		double derivativeTerm = kD * (propTerm - lastPropTerm);
		
		double outputPreSat = propTerm + integralTerm + derivativeTerm + kF*setpoint;
				
		if (outputPreSat < minOutput) {
			result = minOutput;
			saturateLowFlag = true;
			saturateHighFlag = false;
		} else if (outputPreSat > maxOutput) {
			result = maxOutput;
			saturateLowFlag = false;
			saturateHighFlag = true;
		} else {
			result = outputPreSat;
			saturateLowFlag = false;
			saturateHighFlag = false;
		}
		
		saturateError = result - outputPreSat;
		
		lastPropTerm = propTerm;
		
		synchronized (errorBuffer) {
			errorBuffer.add(error);
			while (errorBuffer.size() > bufferLength) {
				errorBuffer.remove();
			}
		}
		
	}
	
	

	@Override
	public void pidWrite(double output) {
		// TODO Auto-generated method stub
		
	}



	
}
