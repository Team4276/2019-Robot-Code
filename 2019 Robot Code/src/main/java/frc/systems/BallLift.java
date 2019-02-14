package frc.systems;

import edu.wpi.first.wpilibj.VictorSP;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.utilities.Xbox;

public class BallLift {

	boolean lowPosition = false;
	boolean highPosition = true;
	double backPowerShoot = 1;
	double backPowerSpit = .4;
	double frontPower = -.7;
	boolean goingHi = false;
	boolean goingLo = false;
	boolean goingOut = false;
	boolean isCollecting = false;

	VictorSPX backroller, frontroller;
	Solenoid diverter;

	public BallLift(int backPort, int frontPort, int divertA) {
		backroller = new VictorSPX(backPort);
		frontroller = new VictorSPX(frontPort);
		diverter = new Solenoid(divertA);
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
		backroller.set(ControlMode.PercentOutput, backPowerSpit);

		goingHi = false;
		goingLo = true;
		goingOut = false;

	}

	public void highScore() {

		diverter.set(highPosition);
		frontroller.set(ControlMode.PercentOutput, frontPower);
		backroller.set(ControlMode.PercentOutput, backPowerShoot);

		goingHi = true;
		goingLo = false;
		goingOut = false;

	}

	public void reverse() {

		frontroller.set(ControlMode.PercentOutput, -frontPower);
		backroller.set(ControlMode.PercentOutput, -backPowerShoot);

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
	public void exStop() {


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
