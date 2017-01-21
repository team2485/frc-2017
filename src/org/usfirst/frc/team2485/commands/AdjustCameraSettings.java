package org.usfirst.frc.team2485.commands;

import org.usfirst.frc.team2485.util.UnifiedCamera;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
* @author Nicholas Contreras
*/

public class AdjustCameraSettings extends InstantCommand {
	private UnifiedCamera camera;
	private int brightness, exposure, whiteBalance, framesPerSecond;
	
	public AdjustCameraSettings(UnifiedCamera camera, int brightness, int exposure, int whiteBalance, int framesPerSecond) {
		this.camera = camera;
		this.brightness = brightness;
		this.exposure = exposure;
		this.whiteBalance = whiteBalance;
		this.framesPerSecond = framesPerSecond;
	}
	
	@Override
	protected void initialize() {
		camera.setSettings(brightness, exposure, whiteBalance, framesPerSecond);
	}
}
