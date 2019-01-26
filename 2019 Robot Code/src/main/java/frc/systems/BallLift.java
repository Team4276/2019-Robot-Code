package frc.systems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Robot;

public class BallLift {

	Value lowPosition = DoubleSolenoid.Value.kForward;
	Value highPosition = DoubleSolenoid.Value.kReverse;
	double backPower = 1;
	double frontPower = 1;
	
	VictorSP backroller, frontroller;
	DoubleSolenoid diverter;
	
	public BallLift(int backPort, int frontPort, int divertA, int divertB) {
		backroller = new VictorSP(backPort);
		frontroller = new VictorSP(frontPort);
		diverter = new DoubleSolenoid(divertA, divertB);
		diverter.set(highPosition);
	}
	
	public void performMainProcessing() {
		
		if(Robot.xboxJoystick.getRawButton(0)) {
			reverse();
		}
		else if(Robot.xboxJoystick.getRawButton(1)){
			lowScore();
		}
		else if(Robot.xboxJoystick.getRawButton(2)){
			highScore();
		}
		else{
			stop();
		}
	}
	
	public void lowScore() {
		
		diverter.set(lowPosition);
		frontroller.set(frontPower);
		backroller.set(backPower);
		
	}
	
	public void highScore() {
		
		diverter.set(highPosition);
		frontroller.set(frontPower);
		backroller.set(backPower);
		
	}
	
	public void reverse() {
		
		frontroller.set(-frontPower);
		backroller.set(-backPower);
		
	}
	
	public void stop() {
		
		frontroller.set(0);
		backroller.set(0);
		
	}
	
}
