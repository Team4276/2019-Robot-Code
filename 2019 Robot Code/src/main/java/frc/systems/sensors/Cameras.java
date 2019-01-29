package frc.systems.sensors;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.cscore.UsbCamera;

public class Cameras {

	UsbCamera mainCamera;
	// UsbCamera armCamera;

	private final int MAIN_RES_X = 352;
	private final int MAIN_RES_Y = 288;
	private final int MAIN_FPS = 25;
	private final int MAIN_EXPOSURE = 30;


	public Cameras() {
		
		mainCamera = CameraServer.getInstance().startAutomaticCapture(0);

		mainCamera.setResolution(MAIN_RES_X, MAIN_RES_Y);
		mainCamera.setFPS(MAIN_FPS);
	    mainCamera.setExposureManual(MAIN_EXPOSURE);
	}

}
