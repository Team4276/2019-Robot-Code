/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.systems;

import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Joystick;

public class DriveSystem {

    VictorSP flDrive, mlDrive, blDrive, frDrive, mrDrive, brDrive;
    Joystick leftJoy;
    Joystick rightJoy;
    double deadband = 0.05;

    public DriveSystem(int port0, int port1, int port2, int port3, int port4, int port5) {
        flDrive = new VictorSP(port0);
        mlDrive = new VictorSP(port1);
        blDrive = new VictorSP(port2);
        frDrive = new VictorSP(port3);
        mrDrive = new VictorSP(port4);
        brDrive = new VictorSP(port5);

    }

    public void assignMotorPower(double rightPow, double leftPow) {
        flDrive.set(leftPow);
        mlDrive.set(leftPow);
        blDrive.set(leftPow);
        frDrive.set(rightPow);
        mrDrive.set(rightPow);
        brDrive.set(rightPow);

    }

    public void operatorDrive() {
        double leftY = 0;
        double rightY = 0;

        if (Math.abs(Robot.leftJoystick.getY()) > deadband) {
            leftY = Math.pow(Robot.leftJoystick.getY(), 3);
        }
        if (Math.abs(Robot.rightJoystick.getY()) > deadband) {
            rightY = Math.pow(Robot.rightJoystick.getY(), 3);
        }
        assignMotorPower(leftY, rightY);

    }

}
