package frc.systems;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.utilities.Xbox;

public class Collector {

    VictorSP motor;
    double intakeSpeed = 1;
    double outtakeSpeed = -1;
    boolean isCollecting = false;

    public Collector(int port1) {
        motor = new VictorSP(port1);
    }

    public void intake() {
        motor.set(intakeSpeed);
        isCollecting = true;
    }

    public void outtake() {
        motor.set(outtakeSpeed);
        isCollecting = true;
    }

    public void stop() {
        motor.set(0);
        isCollecting = false;
    }

    public void performMainProcessing() {
        if (Robot.xboxJoystick.getRawButton(Xbox.LB) || Robot.xboxJoystick.getRawAxis(Xbox.LT) > 0.5) {
            intake();
        } else {
            stop();
        }
        updateTelemetry();
    }

    public void updateTelemetry() {
        SmartDashboard.putBoolean("Collecting:", isCollecting);

    }

}