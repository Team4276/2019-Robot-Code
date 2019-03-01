/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.systems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.utilities.SoftwareTimer;
import frc.utilities.Xbox;

public class ClimbingJack {

    DoubleSolenoid jackSolenoid;
    SoftwareTimer climbTimer;

    Value kLift = Value.kForward;
    Value kRetract = Value.kReverse;
    boolean climbInit = true;
    double pistonDelay = 0.7; // sec

    boolean isClimbing = false;
    boolean isJacked = false;

    public ClimbingJack(int jackA, int jackB) {
        jackSolenoid = new DoubleSolenoid(jackA, jackB);
        climbTimer = new SoftwareTimer();
    }

    public void performMainProcessing() {

        if ((Robot.xboxJoystick.getRawAxis(Xbox.RAxisY) < -0.5) && (Robot.xboxJoystick.getRawButton(Xbox.RAxis))) {
            isClimbing = true;

            if (climbInit) {
                climbTimer.setTimer(pistonDelay);
                climbInit = false;
            }
            Robot.mArmPivot.commandSetpoint(-90);

            if (climbTimer.isExpired()) {
                isJacked = true;
                jackSolenoid.set(kLift);
            } else {
                isJacked = false;
                jackSolenoid.set(kRetract);
            }

        } else {
            climbInit = true;
            isClimbing = false;
            isJacked = false;
            jackSolenoid.set(kRetract);
        }
    }

    public void updateTelemetry() {

        SmartDashboard.putBoolean("Climbing:", isClimbing);
        SmartDashboard.putBoolean("Jacked:", isJacked);
    }

}
