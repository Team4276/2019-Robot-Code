package frc.systems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.utilities.Xbox;

public class BallLift {

	Value lowPosition = DoubleSolenoid.Value.kForward;
	Value highPosition = DoubleSolenoid.Value.kReverse;
	double backPower = 1;
	double frontPower = 1;
	boolean goingHi = false;
	boolean goingLo = false;
	boolean goingOut = false;
	
	VictorSP backroller, frontroller;
	DoubleSolenoid diverter;
	
	public BallLift(int backPort, int frontPort, int divertA, int divertB) {
		backroller = new VictorSP(backPort);
		frontroller = new VictorSP(frontPort);
		diverter = new DoubleSolenoid(divertA, divertB);
		diverter.set(highPosition);
	}
	
	public void performMainProcessing() {
		
		if(Robot.xboxJoystick.getRawButton(Xbox.A)) {
			reverse();
		}
		else if(Robot.xboxJoystick.getRawButton(Xbox.B)){
			lowScore();
		}
		else if(Robot.xboxJoystick.getRawButton(Xbox.Y)){
			highScore();
		}
		else{
			stop();
		}
		updateTelemetry();
	}
	
	public void lowScore() {
		
		diverter.set(lowPosition);
		frontroller.set(frontPower);
		backroller.set(backPower);

		goingHi = false;
		goingLo = true;
		goingOut = false;
		
	}
	
	public void highScore() {
		
		diverter.set(highPosition);
		frontroller.set(frontPower);
		backroller.set(backPower);

		goingHi = true;
		goingLo = false;
		goingOut = false;
		
	}
	
	public void reverse() {
		
		frontroller.set(-frontPower);
		backroller.set(-backPower);
		
		goingHi = false;
		goingLo = false;
		goingOut = true;
		
	}
	
	public void stop() {
		
		frontroller.set(0);
		backroller.set(0);
		
		goingHi = false;
		goingLo = false;
		goingOut = false;
		
	}
	
	public void updateTelemetry(){
		SmartDashboard.putBoolean("High Goal:", goingHi);
		SmartDashboard.putBoolean("Low Goal:", goingLo);
		SmartDashboard.putBoolean("Outake:", goingOut);
	}
	
}
