package org.usfirst.frc.team2485.subsystems;

import org.usfirst.frc.team2485.robot.OI;
import org.usfirst.frc.team2485.robot.RobotMap;
import org.usfirst.frc.team2485.robot.commands.DriveWithControllers;
import org.usfirst.frc.team2485.util.ConstantsIO;
import org.usfirst.frc.team2485.util.PIDSourceWrapper;
import org.usfirst.frc.team2485.util.RampRate;
import org.usfirst.frc.team2485.util.ScalingMax;
import org.usfirst.frc.team2485.util.ThresholdHandler;
import org.usfirst.frc.team2485.util.TransferNode;
import org.usfirst.frc.team2485.util.WarlordsPIDController;

import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * @author Nicholas Contreras
 */

public class DriveTrain extends Subsystem {
	public enum DriveSpeed {
		SLOW_SPEED_RATING, NORMAL_SPEED_RATING;

		public double getSpeedFactor() {

			switch (this) {
			case SLOW_SPEED_RATING:
				return 0.5;
			case NORMAL_SPEED_RATING:
				return 1.0;
			default:
				return 1.0;
			}
		}
	}

	public enum ControlMode {
		OFF, TELEOP_CURRENT, AUTO_CURVE_FOLLOW, AUTO_ROTATE_TO, TEST_VELOCITY_DIRECT, TEST_CURRENT_DIRECT;

		public boolean isAuto() {
			return (this == AUTO_CURVE_FOLLOW || this == AUTO_ROTATE_TO);
		}
	}

	private static final double STEERING_DEADBAND = 0.15;
	private static final double THROTTLE_DEADBAND = 0.01;
	private static final double MIN_CURRENT = 2;
	private static final double MAX_CURRENT = 20;
	private double lastDistError;


	private boolean isQuickTurn;
	private boolean isRotateTo;

	private double driveSpeed = DriveSpeed.NORMAL_SPEED_RATING.getSpeedFactor();

	private WarlordsPIDController velocityPIDRight = new WarlordsPIDController(),
			velocityPIDLeft = new WarlordsPIDController(), rotateToPID = new WarlordsPIDController(),
			distPID = new WarlordsPIDController(), anglePID = new WarlordsPIDController(),
			angVelPIDController = new WarlordsPIDController();
	private TransferNode rotateToTransferNode = new TransferNode(0), overallCurrentTransferNode = new TransferNode(0),
			steeringTransferNode = new TransferNode(0), overallVelocityTransferNode = new TransferNode(0),
			angleSteeringTransferNode = new TransferNode(0), autoCurvatureTransferNode = new TransferNode(0),
			angVelCorrectionTransferNode = new TransferNode(0);
	private PIDSourceWrapper prescaledVelocityLeft = new PIDSourceWrapper(), prescaledVelocityRight = new PIDSourceWrapper(),
			prescaledCurrentRight = new PIDSourceWrapper(), prescaledCurrentLeft = new PIDSourceWrapper(),
			targetAngVelSource = new PIDSourceWrapper(), angVelSource = new PIDSourceWrapper();
	private ScalingMax teleopCurrentScalingMax = new ScalingMax(), velocityScalingMax = new ScalingMax();
	private RampRate overallCurrentRamp = new RampRate(), velocityRampLeft = new RampRate(),
			velocityRampRight = new RampRate(), steeringRamp = new RampRate(), overallVelocityRamp = new RampRate();

	private static final double MIN_SPEED = 1;
	private static final double MAX_SPEED = 180;

	private static final double LOW_SPEED_DRIVETO = 1;
	private static final double LOW_SPEED_ROTATETO = .5;
	public static final double DRIVETO_TOLERANCE = 2;
	private static final double ROTATETO_TOLERANCE = .5;
	
	private boolean errorInAuto;

	public DriveTrain() {

		teleopCurrentScalingMax.setOutputs(RobotMap.driveTrainLeft, RobotMap.driveTrainRight);
		teleopCurrentScalingMax.setSources(prescaledCurrentLeft, prescaledCurrentRight);
		teleopCurrentScalingMax.setSetpoint(40);

		prescaledCurrentRight.setPidSource(() -> {
			if (isQuickTurn) {
				return -MAX_CURRENT * steeringTransferNode.pidGet();
			} else {
				return overallCurrentTransferNode.getOutput() * (1 - steeringTransferNode.getOutput()) - angVelCorrectionTransferNode.getOutput();
			}
		});

		prescaledCurrentLeft.setPidSource(() -> {
			if (isQuickTurn) {
				return MAX_CURRENT * steeringTransferNode.pidGet();
			} else {
				return overallCurrentTransferNode.getOutput() * (1 + steeringTransferNode.getOutput()) + angVelCorrectionTransferNode.getOutput();
			}
		});
		
		angVelSource.setPidSource(() -> {
			return (RobotMap.driveEncLeft.getRate() - RobotMap.driveEncRight.getRate())/RobotMap.ROBOT_WIDTH;
		});
		
		angVelPIDController.setOutputs(angVelCorrectionTransferNode);
		angVelPIDController.setSources(RobotMap.ahrsRateRads);
		angVelPIDController.setSetpointSource(targetAngVelSource);
		
		targetAngVelSource.setPidSource(() -> {
			return steeringTransferNode.getOutput() * 2 / RobotMap.ROBOT_WIDTH * RobotMap.averageEncoderRate.pidGet();
		});

		overallCurrentRamp.setOutputs(overallCurrentTransferNode);
		
		steeringRamp.setOutputs(steeringTransferNode);

		// AUTO
		velocityPIDLeft.setSources(RobotMap.driveEncRateLeft);
		velocityPIDLeft.setOutputs(RobotMap.driveTrainLeft);
		velocityPIDLeft.setOutputRange(-MAX_CURRENT, MAX_CURRENT);

		velocityPIDRight.setSources(RobotMap.driveEncRateRight);
		velocityPIDRight.setOutputs(RobotMap.driveTrainRight);
		velocityPIDRight.setOutputRange(-MAX_CURRENT, MAX_CURRENT);
		
		velocityRampLeft.setOutputs(velocityPIDLeft);
		velocityRampRight.setOutputs(velocityPIDRight);

		velocityScalingMax.setOutputs(velocityRampLeft, velocityRampRight);
		velocityScalingMax.setSources(prescaledVelocityLeft, prescaledVelocityRight);
		velocityScalingMax.setSetpoint(MAX_SPEED);

		prescaledVelocityLeft.setPidSource(() -> {
			if (isRotateTo) {
				return rotateToTransferNode.getOutput();
			} else {
				return overallVelocityTransferNode.getOutput() * (1 + autoCurvatureTransferNode.getOutput()) 
						+ angleSteeringTransferNode.getOutput();
			}
		});

		prescaledVelocityRight.setPidSource(() -> {
			if (isRotateTo) {
				return -rotateToTransferNode.getOutput();
			} else {
				return overallVelocityTransferNode.getOutput() * (1 - autoCurvatureTransferNode.getOutput()) 
						- angleSteeringTransferNode.getOutput();
			}
		});

		anglePID.setSources(RobotMap.ahrs);
		anglePID.setOutputs(angleSteeringTransferNode);
		anglePID.setInputRange(0, 360);
		anglePID.setContinuous(true);
		
		overallVelocityRamp.setOutputs(overallVelocityTransferNode);

		distPID.setSources(RobotMap.averageEncoderDistance);
		distPID.setOutputs(overallVelocityRamp);
		distPID.setAbsoluteTolerance(DRIVETO_TOLERANCE);

		rotateToPID.setSources(RobotMap.ahrs);
		rotateToPID.setOutputs(rotateToTransferNode);
		rotateToPID.setAbsoluteTolerance(ROTATETO_TOLERANCE);
		rotateToPID.setInputRange(0, 360);
		rotateToPID.setOutputRange(-20, 20);
		rotateToPID.setContinuous(true);
		
	}

	public void setDriveSpeed(DriveSpeed speed) {
		driveSpeed = speed.getSpeedFactor();
	}

	public void setQuickTurn(boolean quickTurn) {
		this.isQuickTurn = quickTurn;
	}

	public void setAutoError() {
		errorInAuto = (getDistanceError() > 5);
		System.out.println("Distance Error: " + getDistanceError());
	}
	
	public boolean getAutoError() {
		return errorInAuto;
	}
	
	public void simpleDrive(double throttle, double steering) {
		
		double leftPwm, rightPwm;

		double angularPower;

		if (isQuickTurn) {
			angularPower = steering;
		} else {
			angularPower = throttle * steering;
		}

		rightPwm = leftPwm = throttle;
		leftPwm += angularPower;
		rightPwm -= angularPower;

		if (leftPwm > 1.0) {
			rightPwm -= (leftPwm - 1.0);
			leftPwm = 1.0;
		} else if (rightPwm > 1.0) {
			leftPwm -= (rightPwm - 1.0);
			rightPwm = 1.0;
		} else if (leftPwm < -1.0) {
			rightPwm += (-1.0 - leftPwm);
			leftPwm = -1.0;
		} else if (rightPwm < -1.0) {
			leftPwm += (-1.0 - rightPwm);
			rightPwm = -1.0;
		}

		RobotMap.driveTrainLeft.set(leftPwm);
		RobotMap.driveTrainRight.set(rightPwm);

	}

	public void switchControlMode(ControlMode mode) {
		
		setCurrentMode(mode != ControlMode.OFF);
		teleopCurrentScalingMax.setEnabled(mode == ControlMode.TELEOP_CURRENT);
		angVelPIDController.setEnabled(mode == ControlMode.TELEOP_CURRENT);
		steeringRamp.setEnabled(mode == ControlMode.TELEOP_CURRENT);
		overallCurrentRamp.setEnabled(mode == ControlMode.TELEOP_CURRENT);
		
		velocityPIDLeft.setEnabled(mode.isAuto() || mode == ControlMode.TEST_VELOCITY_DIRECT);
		velocityPIDRight.setEnabled(mode.isAuto() || mode == ControlMode.TEST_VELOCITY_DIRECT);
		velocityRampLeft.setEnabled(mode.isAuto() || mode == ControlMode.TEST_VELOCITY_DIRECT);
		velocityRampRight.setEnabled(mode.isAuto() || mode == ControlMode.TEST_VELOCITY_DIRECT);
		velocityScalingMax.setEnabled(mode.isAuto());
		anglePID.setEnabled(mode == ControlMode.AUTO_CURVE_FOLLOW);
		overallVelocityRamp.setEnabled(mode == ControlMode.AUTO_CURVE_FOLLOW);
		distPID.setEnabled(mode == ControlMode.AUTO_CURVE_FOLLOW);
		rotateToPID.setEnabled(mode == ControlMode.AUTO_ROTATE_TO);
		isRotateTo = (mode == ControlMode.AUTO_ROTATE_TO);
	}

	/**
	 * W.A.R. Lord Drive This drive method is based off of Team 254's Ultimate
	 * Ascent cheesyDrive code.
	 *
	 * @param controllerY
	 *            controllerY should be positive for forward motion
	 * @param controllerX
	 */
	public void warlordDrive(double controllerY, double controllerX, boolean simple) {
		
		switchControlMode(simple ? ControlMode.OFF : ControlMode.TELEOP_CURRENT);

		if (simple) {
			
			double steering = ThresholdHandler.deadbandAndScale(controllerX, STEERING_DEADBAND, 0.0, 1);
			double throttle = ThresholdHandler.deadbandAndScale(controllerY, THROTTLE_DEADBAND, 0.02, 1);
			throttle *= driveSpeed;
			
			simpleDrive(throttle, steering);
			
		} else {
			double steering = ThresholdHandler.deadbandAndScale(controllerX, STEERING_DEADBAND, 0.0, 0.75) + ThresholdHandler.deadbandAndScale(OI.ben.getRawAxis(OI.XBOX_AXIS_RX), STEERING_DEADBAND, 0, 1);
			System.out.println("steering:" + steering);
			double overallCurrent = ThresholdHandler.deadbandAndScale(controllerY, THROTTLE_DEADBAND, MIN_CURRENT, MAX_CURRENT);
			overallCurrent *= driveSpeed;
//			if (!isQuickTurn)
//				steering *= 0.5;
			
			overallCurrentRamp.setSetpoint(overallCurrent);
			steeringRamp.setSetpoint(steering);

			double averageSpeed = (RobotMap.driveEncLeft.getRate() + RobotMap.driveEncRight.getRate()) / 2;
			if (Math.abs(averageSpeed) > MIN_SPEED && !isQuickTurn) {
				angVelPIDController.enable();
			} else {
				angVelPIDController.disable();
				angVelCorrectionTransferNode.setOutput(0);
			}
			
		}
		
	}

	public void zeroEncoders() {
		
		RobotMap.driveEncLeft.reset();
		RobotMap.driveEncRight.reset();

	}

	public void setCurrentMode(boolean isCurrent) {
		
		TalonControlMode mode = isCurrent ? TalonControlMode.Current : TalonControlMode.PercentVbus;
		
		RobotMap.driveLeft1.changeControlMode(mode);
		RobotMap.driveLeft2.changeControlMode(mode);
		RobotMap.driveLeft3.changeControlMode(mode);
		RobotMap.driveRight1.changeControlMode(mode);
		RobotMap.driveRight2.changeControlMode(mode);
		RobotMap.driveRight3.changeControlMode(mode);
		
	}
	
	public double getAngularVelocityError() {
		return angVelPIDController.getAvgError();
	}

	public double getLeftVelocityPIDError() {
		return velocityPIDLeft.getAvgError();
	}

	public double getAnglePIDError() {
		return anglePID.getAvgError();
	}

	public void updateConstants() {
		
		RobotMap.driveLeft1.setPID(ConstantsIO.kP_DriveCurrent, ConstantsIO.kI_DriveCurrent,
				ConstantsIO.kD_DriveCurrent, ConstantsIO.kF_DriveCurrent, 0, 0, 0);
		RobotMap.driveLeft2.setPID(ConstantsIO.kP_DriveCurrent, ConstantsIO.kI_DriveCurrent,
				ConstantsIO.kD_DriveCurrent, ConstantsIO.kF_DriveCurrent, 0, 0, 0);
		RobotMap.driveLeft3.setPID(ConstantsIO.kP_DriveCurrent, ConstantsIO.kI_DriveCurrent,
				ConstantsIO.kD_DriveCurrent, ConstantsIO.kF_DriveCurrent, 0, 0, 0);
		RobotMap.driveRight1.setPID(ConstantsIO.kP_DriveCurrent, ConstantsIO.kI_DriveCurrent,
				ConstantsIO.kD_DriveCurrent, ConstantsIO.kF_DriveCurrent, 0, 0, 0);
		RobotMap.driveRight2.setPID(ConstantsIO.kP_DriveCurrent, ConstantsIO.kI_DriveCurrent,
				ConstantsIO.kD_DriveCurrent, ConstantsIO.kF_DriveCurrent, 0, 0, 0);
		RobotMap.driveRight3.setPID(ConstantsIO.kP_DriveCurrent, ConstantsIO.kI_DriveCurrent,
				ConstantsIO.kD_DriveCurrent, ConstantsIO.kF_DriveCurrent, 0, 0, 0);
		
		angVelPIDController.setPID(ConstantsIO.kP_DriveAngVel, ConstantsIO.kI_DriveAngVel, ConstantsIO.kD_DriveAngVel);
		velocityPIDRight.setPID(ConstantsIO.kP_DriveVelocity, ConstantsIO.kI_DriveVelocity,
				ConstantsIO.kD_DriveVelocity, ConstantsIO.kF_DriveVelocity);
		velocityPIDLeft.setPID(ConstantsIO.kP_DriveVelocity, ConstantsIO.kI_DriveVelocity, ConstantsIO.kD_DriveVelocity,
				ConstantsIO.kF_DriveVelocity);
		rotateToPID.setPID(ConstantsIO.kP_RotateTo, ConstantsIO.kI_RotateTo, ConstantsIO.kD_RotateTo);
		distPID.setPID(ConstantsIO.kP_Distance, ConstantsIO.kI_Distance, ConstantsIO.kD_Distance);
		anglePID.setPID(ConstantsIO.kP_DriveAngle, ConstantsIO.kI_DriveAngle, ConstantsIO.kD_DriveAngle);
		
		overallCurrentRamp.setRampRates(ConstantsIO.kUpRamp_DriveThrottle, ConstantsIO.kDownRamp_DriveThrottle);
		velocityRampLeft.setRampRates(ConstantsIO.kUpRamp_IndividualVelocity, ConstantsIO.kDownRamp_IndividualVelocity);
		velocityRampRight.setRampRates(ConstantsIO.kUpRamp_IndividualVelocity, ConstantsIO.kDownRamp_IndividualVelocity);
		overallVelocityRamp.setRampRates(ConstantsIO.kUpRamp_OverallVelocity, ConstantsIO.kDownRamp_OverallVelocity);
		steeringRamp.setRampRates(ConstantsIO.kUpRamp_DriveSteering, ConstantsIO.kDownRamp_DriveSteering);
		
	}

	public void reset() {
		
		switchControlMode(ControlMode.OFF);
		RobotMap.driveTrainLeft.set(0);
		RobotMap.driveTrainRight.set(0);
		
	}

	public void setLeftRightVelocity(double l, double r) {
		
		switchControlMode(ControlMode.TEST_VELOCITY_DIRECT);
		velocityRampLeft.setSetpoint(l);
		velocityRampRight.setSetpoint(r);
		
	}
	
	public void setLeftRightCurrent(double l, double r) {
		
		switchControlMode(ControlMode.TEST_CURRENT_DIRECT);
		RobotMap.driveLeft1.set(l);
		RobotMap.driveLeft2.set(l);
		RobotMap.driveLeft3.set(1);
		RobotMap.driveRight1.set(r);
		RobotMap.driveRight2.set(r);
		RobotMap.driveRight3.set(r);
	}

	public boolean driveTo(double distance, double maxSpeed, double angle, double curvature, double tolerance) {
		
		switchControlMode(ControlMode.AUTO_CURVE_FOLLOW);
		distPID.setSetpoint(distance);
		distPID.setOutputRange(-maxSpeed, maxSpeed);
		anglePID.setSetpoint(angle);
		anglePID.setOutputRange(-maxSpeed, maxSpeed);
		autoCurvatureTransferNode.setOutput(curvature);
		distPID.setAbsoluteTolerance(tolerance);
		lastDistError = distPID.getAvgError();
		return (distPID.isOnTarget() && Math.abs((RobotMap.driveEncRight.getRate() + 
				RobotMap.driveEncLeft.getRate()) / 2) < LOW_SPEED_DRIVETO);
		
	}

	public boolean rotateTo(double angle) {
		
		switchControlMode(ControlMode.AUTO_ROTATE_TO);
		rotateToPID.setSetpoint(angle);
		anglePID.setSetpoint(angle);
		return (rotateToPID.isOnTarget() && Math.abs(RobotMap.ahrs.getRate()) < LOW_SPEED_ROTATETO);
		
	}
	
	public double getDistanceError() {
		return lastDistError;
	}
	
	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new DriveWithControllers(false));
	}
	
}
