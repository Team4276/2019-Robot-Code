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

    public Collector() {
    }

    public void intake() {
        Robot.mBallLift.collect();
        isCollecting = true;
    }

    public void outtake() {
        Robot.mBallLift.reverse();
        isCollecting = true;
    }

    public void stop() {
        Robot.mBallLift.stop();
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