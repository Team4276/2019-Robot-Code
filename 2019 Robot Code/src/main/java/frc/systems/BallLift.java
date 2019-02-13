package frc.systems;

import edu.wpi.first.wpilibj.VictorSP;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.utilities.Xbox;

public class BallLift {

	Value lowPosition = DoubleSolenoid.Value.kForward;
	Value highPosition = DoubleSolenoid.Value.kReverse;
	double backPower = 1;
	double frontPower = -1;
	boolean goingHi = false;
	boolean goingLo = false;
	boolean goingOut = false;
	boolean isCollecting = false;

	VictorSPX backroller, frontroller;
	DoubleSolenoid diverter;

	public BallLift(int backPort, int frontPort, int divertA, int divertB) {
		backroller = new VictorSPX(backPort);
		frontroller = new VictorSPX(frontPort);
		diverter = new DoubleSolenoid(divertA, divertB);
		diverter.set(highPosition);
	}

	public void performMainProcessing() {

		if (isCollecting) {
			collect(frontPower);
		} else if (Robot.xboxJoystick.getRawButton(Xbox.A)) {
			reverse();
		} else if (Robot.xboxJoystick.getRawButton(Xbox.B)) {
			lowScore();
		} else if (Robot.xboxJoystick.getRawButton(Xbox.Y)) {
			highScore();
		} else {
			stop();
		}
		updateTelemetry();
	}

	public void lowScore() {

		diverter.set(lowPosition);
		frontroller.set(ControlMode.PercentOutput, frontPower);
		backroller.set(ControlMode.PercentOutput, backPower);

		goingHi = false;
		goingLo = true;
		goingOut = false;

	}

	public void highScore() {

		diverter.set(highPosition);
		frontroller.set(ControlMode.PercentOutput, frontPower);
		backroller.set(ControlMode.PercentOutput, backPower);

		goingHi = true;
		goingLo = false;
		goingOut = false;

	}

	public void reverse() {

		frontroller.set(ControlMode.PercentOutput, -frontPower);
		backroller.set(ControlMode.PercentOutput, -backPower);

		goingHi = false;
		goingLo = false;
		goingOut = true;

	}

	public void stop() {

		frontroller.set(ControlMode.PercentOutput, 0);
		backroller.set(ControlMode.PercentOutput, 0);

		goingHi = false;
		goingLo = false;
		goingOut = false;
		isCollecting = false;
	}

	private void collect(double power) {
		frontroller.set(ControlMode.PercentOutput, power);

	}

	public void collect() {
		isCollecting = true;
	}

	public void updateTelemetry() {
		SmartDashboard.putBoolean("High Goal:", goingHi);
		SmartDashboard.putBoolean("Low Goal:", goingLo);
		SmartDashboard.putBoolean("Outake:", goingOut);
	}

}
