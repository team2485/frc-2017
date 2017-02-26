package org.usfirst.frc.team2485.subsystems;

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
		OFF, TELEOP_CHEESY, TELEOP_CURRENT, AUTO_CURVE_FOLLOW, AUTO_ROTATE_TO, TEST_VELOCITY_DIRECT, TEST_CURRENT_DIRECT;

		public boolean isTeleop() {
			return (this == TELEOP_CHEESY  || this == TELEOP_CURRENT);
		}

		public boolean isAuto() {
			return (this == AUTO_CURVE_FOLLOW || this == AUTO_ROTATE_TO);
		}
	}

//	private static final boolean USE_GYRO_STEERING_CORRECTION = false;
	private static final double STEERING_DEADBAND = 0.15;
	private static final double THROTTLE_DEADBAND = 0.01;
	private static final double MIN_CURRENT = 2;
	private static final double MAX_CURRENT = 20;


	private boolean isQuickTurn;
	private boolean isRotateTo;

	private double driveSpeed = DriveSpeed.NORMAL_SPEED_RATING.getSpeedFactor();

	private double oldSteering, negInertiaAccumulator, quickStopAccumulator;

	private WarlordsPIDController velocityPIDRight = new WarlordsPIDController(),
			velocityPIDLeft = new WarlordsPIDController(), rotateToPID = new WarlordsPIDController(),
			distPID = new WarlordsPIDController(), anglePID = new WarlordsPIDController(),
			steeringPIDController = new WarlordsPIDController(), angVelPIDController = new WarlordsPIDController();
	private TransferNode rotateToTransferNode = new TransferNode(0), overallCurrentTransferNode = new TransferNode(0),
			steeringTransferNode = new TransferNode(0), overallVelocityTransferNode = new TransferNode(0),
			angleSteeringTransferNode = new TransferNode(0), autoCurvatureTransferNode = new TransferNode(0),
			angVelCorrectionTransferNode = new TransferNode(0);
	private PIDSourceWrapper curvatureSource = new PIDSourceWrapper(), 
			prescaledVelocityLeft = new PIDSourceWrapper(), prescaledVelocityRight = new PIDSourceWrapper(),
			prescaledCurrentRight = new PIDSourceWrapper(), prescaledCurrentLeft = new PIDSourceWrapper(),
			targetAngVelSource = new PIDSourceWrapper(), angVelSource = new PIDSourceWrapper();

	private ScalingMax teleopCurrentScalingMax = new ScalingMax(), velocityScalingMax = new ScalingMax();
	private RampRate overallCurrentRamp = new RampRate(ConstantsIO.kUpRamp_DriveThrottle, ConstantsIO.kDownRamp_DriveThrottle),
			velocityRampLeft = new RampRate(ConstantsIO.kUpRamp_IndividualVelocity,
					ConstantsIO.kDownRamp_IndividualVelocity),
			velocityRampRight = new RampRate(ConstantsIO.kUpRamp_IndividualVelocity,
					ConstantsIO.kDownRamp_IndividualVelocity),
			steeringRamp = new RampRate(ConstantsIO.kUpRamp_DriveSteering, ConstantsIO.kDownRamp_DriveSteering),
			overallVelocityRampRate = new RampRate(ConstantsIO.kUpRamp_OverallVelocity,
					ConstantsIO.kDownRamp_OverallVelocity);
//	private PIDOutputWrapper motorModeSwitcherLeft = new PIDOutputWrapper(),
//			motorModeSwitcherRight = new PIDOutputWrapper();

	private static final double MIN_SPEED = 1;
	private static final double MAX_SPEED = 180;

	private static final double LOW_SPEED_DRIVETO = 1;
	private static final double LOW_SPEED_ROTATETO = .5;
	private static final double DRIVETO_TOLERANCE = 2;
	private static final double ROTATETO_TOLERANCE = .5;

	public DriveTrain() {

//		motorModeSwitcherLeft.setPidOutput((double out) -> {
//			if (Math.abs(out * MAX_CURRENT) > MIN_CURRENT && useCurrent) {
//				setCurrentMode(true);
//				RobotMap.driveTrainLeft.set(out * MAX_CURRENT);
//			} else {
//				setCurrentMode(false);
//				RobotMap.driveTrainLeft.set(out);
//			}
//		});
//
//		motorModeSwitcherRight.setPidOutput((double out) -> {
//			if (Math.abs(out * MAX_CURRENT) > MIN_CURRENT && useCurrent) {
//				setCurrentModeRight(true);
//				RobotMap.driveTrainRight.set(out * MAX_CURRENT);
//			} else {
//				setCurrentModeRight(false);
//				RobotMap.driveTrainRight.set(out);
//			}
//		});

		teleopCurrentScalingMax.setOutputs(RobotMap.driveTrainLeft, RobotMap.driveTrainRight);
		teleopCurrentScalingMax.setSources(prescaledCurrentLeft, prescaledCurrentRight);
		teleopCurrentScalingMax.setSetpoint(40);

		prescaledCurrentRight.setPidSource(() -> {
			if (isQuickTurn) {
				return -MAX_CURRENT*steeringTransferNode.pidGet();
			} else {
				return overallCurrentTransferNode.getOutput() * (1 - steeringTransferNode.getOutput()) - angVelCorrectionTransferNode.getOutput();
			}
		});

		prescaledCurrentLeft.setPidSource(() -> {
			if (isQuickTurn) {
				return MAX_CURRENT*steeringTransferNode.pidGet();
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
			return steeringTransferNode.getOutput()  * 2 / RobotMap.ROBOT_WIDTH * RobotMap.averageEncoderRate.pidGet();
		});

		overallCurrentRamp.setOutputs(overallCurrentTransferNode);
		steeringRamp.setOutputs(steeringTransferNode);

		// AUTO
		velocityPIDLeft.setSources(RobotMap.driveEncRateLeft);
		velocityPIDLeft.setOutputs(RobotMap.driveTrainLeft);
		velocityPIDRight.setSources(RobotMap.driveEncRateRight);
		velocityPIDRight.setOutputs(RobotMap.driveTrainRight);
		
		velocityPIDLeft.setOutputRange(-MAX_CURRENT, MAX_CURRENT);
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
				return overallVelocityTransferNode.getOutput() * (1 + autoCurvatureTransferNode.getOutput()) + angleSteeringTransferNode.getOutput();
			}
		});

		prescaledVelocityRight.setPidSource(() -> {
			if (isRotateTo) {
				return -rotateToTransferNode.getOutput();
			} else {
				return overallVelocityTransferNode.getOutput() * (1 - autoCurvatureTransferNode.getOutput()) - angleSteeringTransferNode.getOutput();
			}
		});


		anglePID.setSources(RobotMap.ahrs);
		anglePID.setOutputs(angleSteeringTransferNode);

		overallVelocityRampRate.setOutputs(overallVelocityTransferNode);

		distPID.setSources(RobotMap.averageEncoderDistance);
		distPID.setOutputs(overallVelocityRampRate);
		distPID.setAbsoluteTolerance(DRIVETO_TOLERANCE);

		rotateToPID.setSources(RobotMap.ahrs);
		rotateToPID.setOutputs(rotateToTransferNode);
		rotateToPID.setAbsoluteTolerance(ROTATETO_TOLERANCE);
		rotateToPID.setInputRange(0, 360);
		rotateToPID.setOutputRange(-10, 10);
		rotateToPID.setContinuous(true);
	}

	public void setDriveSpeed(DriveSpeed speed) {
		driveSpeed = speed.getSpeedFactor();
	}

	public double getDriveSpeed() {
		return driveSpeed;
	}

	public void setQuickTurn(boolean quickTurn) {
		this.isQuickTurn = quickTurn;
	}

	public void cheesyDrive(double controllerY, double controllerX) {
		double steeringNonLinearity;

		double steering = ThresholdHandler.deadbandAndScale(controllerX, STEERING_DEADBAND, 0.01, 1);
		double throttle = ThresholdHandler.deadbandAndScale(controllerY, THROTTLE_DEADBAND, 0.01, 1);

		double negInertia = steering - oldSteering;
		oldSteering = steering;

		steeringNonLinearity = 0.5;
		// Apply a sin function that's scaled to make it feel better.
		steering = Math.sin(Math.PI / 2.0 * steeringNonLinearity * steering)
				/ Math.sin(Math.PI / 2.0 * steeringNonLinearity);
		steering = Math.sin(Math.PI / 2.0 * steeringNonLinearity * steering)
				/ Math.sin(Math.PI / 2.0 * steeringNonLinearity);
		steering = Math.sin(Math.PI / 2.0 * steeringNonLinearity * steering)
				/ Math.sin(Math.PI / 2.0 * steeringNonLinearity);

		double leftPwm, rightPwm, overPower;
		double sensitivity;

		double angularPower;
		double linearPower;

		// Negative inertia!
		double negInertiaScalar;

		if (steering * negInertia > 0) {
			negInertiaScalar = 2.5;
		} else {
			if (Math.abs(steering) > 0.65) {
				negInertiaScalar = 5.0;
			} else {
				negInertiaScalar = 3.0;
			}
		}
		sensitivity = .75;

		double negInertiaPower = negInertia * negInertiaScalar;
		negInertiaAccumulator += negInertiaPower;

		steering = steering + negInertiaAccumulator;
		if (negInertiaAccumulator > 1) {
			negInertiaAccumulator -= 1;
		} else if (negInertiaAccumulator < -1) {
			negInertiaAccumulator += 1;
		} else {
			negInertiaAccumulator = 0;
		}
		linearPower = throttle * driveSpeed;

		// Quickturn!
		if (isQuickTurn) {
			if (Math.abs(linearPower) < 0.2) {
				double alpha = 0.1;
				quickStopAccumulator = (1 - alpha) * quickStopAccumulator + alpha * steering * 5;
			}
			overPower = 1.0;

			sensitivity = 1.0;

			angularPower = steering;
		} else {
			overPower = 0.0;
			angularPower = Math.abs(throttle) * steering * sensitivity - quickStopAccumulator;
			if (quickStopAccumulator > 1) {
				quickStopAccumulator -= 1;
			} else if (quickStopAccumulator < -1) {
				quickStopAccumulator += 1;
			} else {
				quickStopAccumulator = 0.0;
			}
		}

		rightPwm = leftPwm = linearPower;
		leftPwm += angularPower;
		rightPwm -= angularPower;

		if (leftPwm > 1.0) {
			rightPwm -= overPower * (leftPwm - 1.0);
			leftPwm = 1.0;
		} else if (rightPwm > 1.0) {
			leftPwm -= overPower * (rightPwm - 1.0);
			rightPwm = 1.0;
		} else if (leftPwm < -1.0) {
			rightPwm += overPower * (-1.0 - leftPwm);
			leftPwm = -1.0;
		} else if (rightPwm < -1.0) {
			leftPwm += overPower * (-1.0 - rightPwm);
			rightPwm = -1.0;
		}

		RobotMap.driveTrainLeft.set(leftPwm);
		RobotMap.driveTrainRight.set(rightPwm);

	}

	public void switchControlMode(ControlMode mode) {
		setCurrentMode(mode != ControlMode.OFF && mode != ControlMode.TELEOP_CHEESY);
		teleopCurrentScalingMax.setEnabled(mode == ControlMode.TELEOP_CURRENT);
		angVelPIDController.setEnabled(mode == ControlMode.TELEOP_CURRENT);
//		steeringPIDController.setEnabled(mode == ControlMode.TELEOP_CURRENT || mode == ControlMode.TELEOP_VOLTAGE);
		steeringRamp.setEnabled(mode.isTeleop());
		overallCurrentRamp.setEnabled(mode.isTeleop());
		velocityPIDLeft.setEnabled(
				mode.isAuto() || mode == ControlMode.TEST_VELOCITY_DIRECT);
		velocityPIDRight.setEnabled(
				mode.isAuto() || mode == ControlMode.TEST_VELOCITY_DIRECT);
		velocityRampLeft.setEnabled(
				mode.isAuto() || mode == ControlMode.TEST_VELOCITY_DIRECT);
		velocityRampRight.setEnabled(
				mode.isAuto() || mode == ControlMode.TEST_VELOCITY_DIRECT);
		velocityScalingMax.setEnabled(mode.isAuto());
		anglePID.setEnabled(mode == ControlMode.AUTO_CURVE_FOLLOW);
		overallVelocityRampRate.setEnabled(mode == ControlMode.AUTO_CURVE_FOLLOW);
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
	public void warlordDrive(double controllerY, double controllerX, ControlMode mode) {
		
		switchControlMode(mode);

		if (mode == ControlMode.TELEOP_CHEESY) {
			double steering = ThresholdHandler.deadbandAndScale(controllerX, STEERING_DEADBAND, 0.0, 1);
			double throttle = ThresholdHandler.deadbandAndScale(controllerY, THROTTLE_DEADBAND, 0.02, 1);

			throttle *= driveSpeed;
			cheesyDrive(throttle, steering);
		} else {
			double steering = ThresholdHandler.deadbandAndScale(controllerX, STEERING_DEADBAND, 0.0, 1);
			double overallCurrent = ThresholdHandler.deadbandAndScale(controllerY, THROTTLE_DEADBAND, MIN_CURRENT, MAX_CURRENT);

			overallCurrent *= driveSpeed;
			overallCurrentRamp.setSetpoint(overallCurrent);
			steeringRamp.setSetpoint(steering);

			double averageSpeed = (RobotMap.driveEncLeft.getRate() + RobotMap.driveEncRight.getRate()) / 2;

			if (Math.abs(averageSpeed) > MIN_SPEED && !isQuickTurn && mode == ControlMode.TELEOP_CURRENT) {
//				steeringRamp.setOutputs(steeringPIDController);
				angVelPIDController.enable();
			} else {
//				steeringRamp.setOutputs(steeringTransferNode);
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
		if (isCurrent) {
			RobotMap.driveLeft1.changeControlMode(TalonControlMode.Current);
			RobotMap.driveLeft2.changeControlMode(TalonControlMode.Current);
			RobotMap.driveRight1.changeControlMode(TalonControlMode.Current);
			RobotMap.driveRight2.changeControlMode(TalonControlMode.Current);
		} else {
			RobotMap.driveLeft1.changeControlMode(TalonControlMode.PercentVbus);
			RobotMap.driveLeft2.changeControlMode(TalonControlMode.PercentVbus);
			RobotMap.driveRight1.changeControlMode(TalonControlMode.PercentVbus);
			RobotMap.driveRight2.changeControlMode(TalonControlMode.PercentVbus);
		}
	}

	public double getCurvature() {
		return curvatureSource.pidGet();
	}
	
	public double getAngularVelocityError() {
		return angVelPIDController.getAvgError();
	}

	public double getSteering() {
		return steeringPIDController.getSetpoint();
	}

	public double getCurvatureError() {

		return steeringPIDController.getAvgError();
	}

	public double getLeftVelocityPIDError() {
		return velocityPIDLeft.getAvgError();
	}

	public double getAnglePIDError() {
		return anglePID.getAvgError();
	}

	public void updateConstants() {
		angVelPIDController.setPID(ConstantsIO.kP_DriveAngVel, ConstantsIO.kI_DriveAngVel, ConstantsIO.kD_DriveAngVel);
		steeringPIDController.setPID(ConstantsIO.kP_DriveSteering, ConstantsIO.kI_DriveSteering,
				ConstantsIO.kD_DriveSteering, ConstantsIO.kF_DriveSteering);
		velocityPIDRight.setPID(ConstantsIO.kP_DriveVelocity, ConstantsIO.kI_DriveVelocity,
				ConstantsIO.kD_DriveVelocity, ConstantsIO.kF_DriveVelocity);
		velocityPIDLeft.setPID(ConstantsIO.kP_DriveVelocity, ConstantsIO.kI_DriveVelocity, ConstantsIO.kD_DriveVelocity,
				ConstantsIO.kF_DriveVelocity);
		overallCurrentRamp.setRampRates(ConstantsIO.kUpRamp_DriveThrottle, ConstantsIO.kDownRamp_DriveThrottle);
		velocityRampLeft.setRampRates(ConstantsIO.kUpRamp_IndividualVelocity,
				ConstantsIO.kDownRamp_IndividualVelocity);
		velocityRampRight.setRampRates(ConstantsIO.kUpRamp_IndividualVelocity,
				ConstantsIO.kDownRamp_IndividualVelocity);
		rotateToPID.setPID(ConstantsIO.kP_RotateTo, ConstantsIO.kI_RotateTo, ConstantsIO.kD_RotateTo,
				ConstantsIO.kF_RotateTo);
		RobotMap.driveLeft1.setPID(ConstantsIO.kP_DriveCurrentCIM, ConstantsIO.kI_DriveCurrentCIM,
				ConstantsIO.kD_DriveCurrentCIM, ConstantsIO.kF_DriveCurrentCIM, 0, 0, 0);
		RobotMap.driveLeft2.setPID(ConstantsIO.kP_DriveCurrentCIM, ConstantsIO.kI_DriveCurrentCIM,
				ConstantsIO.kD_DriveCurrentCIM, ConstantsIO.kF_DriveCurrentCIM, 0, 0, 0);
		RobotMap.driveRight1.setPID(ConstantsIO.kP_DriveCurrentCIM, ConstantsIO.kI_DriveCurrentCIM,
				ConstantsIO.kD_DriveCurrentCIM, ConstantsIO.kF_DriveCurrentCIM, 0, 0, 0);
		RobotMap.driveRight2.setPID(ConstantsIO.kP_DriveCurrentCIM, ConstantsIO.kI_DriveCurrentCIM,
				ConstantsIO.kD_DriveCurrentCIM, ConstantsIO.kF_DriveCurrentCIM, 0, 0, 0);
		overallVelocityRampRate.setRampRates(ConstantsIO.kUpRamp_OverallVelocity,
				ConstantsIO.kDownRamp_OverallVelocity);
		steeringRamp.setRampRates(ConstantsIO.kUpRamp_DriveSteering, ConstantsIO.kDownRamp_DriveSteering);
		distPID.setPID(ConstantsIO.kP_Distance, ConstantsIO.kI_Distance, ConstantsIO.kD_Distance,
				ConstantsIO.kF_Distance);
		anglePID.setPID(ConstantsIO.kP_DriveAngle, ConstantsIO.kI_DriveAngle, ConstantsIO.kD_DriveAngle);
	}

	@Override
	protected void initDefaultCommand() {
		setDefaultCommand(new DriveWithControllers(ControlMode.TELEOP_CURRENT));
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
		
		System.out.println("Velocity PID Left setpoint: " + velocityPIDLeft.getSetpoint());
	}
	
	public void setLeftRightCurrent(double l, double r) {
		switchControlMode(ControlMode.TEST_CURRENT_DIRECT);
		RobotMap.driveLeft1.set(l);
		RobotMap.driveLeft2.set(l);
		RobotMap.driveRight1.set(r);
		RobotMap.driveRight2.set(r);
	}

	public boolean driveTo(double distance, double maxSpeed, double angle, double curvature) {
		switchControlMode(ControlMode.AUTO_CURVE_FOLLOW);
		distPID.setSetpoint(distance);
		distPID.setOutputRange(-maxSpeed, maxSpeed);
		anglePID.setSetpoint(angle);
		anglePID.setOutputRange(-maxSpeed, maxSpeed);
		return (distPID.isOnTarget() && Math.abs((RobotMap.driveEncRight.getRate() + 
				RobotMap.driveEncLeft.getRate()) / 2) < LOW_SPEED_DRIVETO);
	}

	public boolean rotateTo(double angle) {
		switchControlMode(ControlMode.AUTO_ROTATE_TO);
		rotateToPID.setSetpoint(angle);
		anglePID.setSetpoint(angle);
		return (rotateToPID.isOnTarget() && Math.abs(RobotMap.ahrs.getRate()) < LOW_SPEED_ROTATETO);
	}
}
