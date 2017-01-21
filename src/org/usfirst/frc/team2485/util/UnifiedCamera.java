package org.usfirst.frc.team2485.util;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

import edu.wpi.first.wpilibj.vision.CameraServer;
import edu.wpi.first.wpilibj.vision.USBCamera;

/**
 * @author Nicholas Contreras
 */

public class UnifiedCamera {

	private VideoCapture cvCamera;
	
	private USBCamera niCamera;

	private Size imageRes;

	private Size imageSize;
	
	/**
	 * Construct a UnifiedCamera and allow the camera to choose the image size and resolution
	 */
	public UnifiedCamera(int camNumber) {
			cvCamera = new VideoCapture(camNumber);
			
			niCamera = new USBCamera("cam" + camNumber);
			
			if (cvCamera.isOpened()) {
				cvCamera.release();
			}

			try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}

			cvCamera.open(0);
			
			Mat pic = getSampleMat();
			
			imageSize = pic.size();
			
			imageRes = pic.size();

			System.out.println("Camera Ready");
		}
	
	/**
	 * Construct a UnifiedCamera with parameters for size and resolution specified
	 * @param imageSize the size of the image
	 * @param imageRes the resolution of the image
	 */

	public UnifiedCamera(int camNumber, Size imageSize, Size imageRes) {
			this(camNumber);
			
			this.imageSize = imageSize;
			
			this.imageRes = imageRes;
			
		}

	public Size getImageRes() {
		return imageRes;
	}

	private Mat getSampleMat() {
		Mat frame = new Mat();

		cvCamera.read(frame);

		return frame;
	}

	public Mat getMat() {
		Mat frame = new Mat();

		cvCamera.read(frame);

		Mat newFrame = new Mat();

		Imgproc.resize(frame, newFrame, imageRes);

		Imgproc.resize(newFrame, frame, imageSize);

		return frame;
	}

	public Size getImageSize() {
		return imageSize;
	}

	public void shutoff() {
		cvCamera.release();
	}

	public void changeRes(Size newRes) {
		imageRes = newRes;
	}
	
	public void setSettings(int brightness, int exposure, int whiteBalance, int framesPerSecond) {
		niCamera.setBrightness(brightness);
		niCamera.setExposureManual(exposure);
		niCamera.setWhiteBalanceManual(whiteBalance);
		niCamera.setFPS(framesPerSecond);
		niCamera.updateSettings();
	}
	
	public void setAutoSettings(){
		niCamera.setWhiteBalanceAuto();
		niCamera.setExposureAuto();
		niCamera.updateSettings();
	}
	
	public void holdCurrentSettings(){
		niCamera.setExposureHoldCurrent();
		niCamera.setWhiteBalanceHoldCurrent();
		niCamera.updateSettings();
	}
	
	public void streamToSmartDashboard(){
		CameraServer.getInstance().startAutomaticCapture(niCamera);
	}

}
