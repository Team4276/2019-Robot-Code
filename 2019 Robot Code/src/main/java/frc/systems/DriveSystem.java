/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.systems;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.Encoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;

import frc.utilities.Constants;
import frc.utilities.SoftwareTimer;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;
import jaci.pathfinder.followers.EncoderFollower;

public class DriveSystem {

    private EncoderFollower leftFollower;
    private EncoderFollower rightFollower;

    private Encoder leftBaseEncoder;
    private Encoder rightBaseEncoder;

    private boolean hasCANNetwork = false;

    DoubleSolenoid gearShifter;
    public final int HI_SHIFTER = 4;
    public final int LO_SHIFTER = 3;
    SoftwareTimer shiftTimer;
    private boolean shiftInit = true;
    private boolean isShifting = false;
    private Gear currentGear;

    VictorSP flDrive, mlDrive, blDrive, frDrive, mrDrive, brDrive;
    VictorSPX flDriveX, mlDriveX, blDriveX, frDriveX, mrDriveX, brDriveX;
    double deadband = 0.05;

    public DriveSystem(boolean isCAN, int FLport, int MLport, int BLport, int FRport, int MRport, int BRport,
            int shifterHi, int shifterLo) {
        if (isCAN) {
            hasCANNetwork = true;

            flDriveX = new VictorSPX(FLport);
            mlDriveX = new VictorSPX(MLport);
            blDriveX = new VictorSPX(BLport);
            frDriveX = new VictorSPX(FRport);
            mrDriveX = new VictorSPX(MRport);
            brDriveX = new VictorSPX(BRport);
        } else {
            hasCANNetwork = false;

            flDrive = new VictorSP(FLport);
            mlDrive = new VictorSP(MLport);
            blDrive = new VictorSP(BLport);
            frDrive = new VictorSP(FRport);
            mrDrive = new VictorSP(MRport);
            brDrive = new VictorSP(BRport);
        }

        gearShifter = new DoubleSolenoid(shifterHi, shifterLo);

    }

    /**
     * 
     * @param rightPow right motor power
     * @param leftPow  left motor power
     */

    public void assignMotorPower(double rightPow, double leftPow) {
        if (hasCANNetwork) {
            flDriveX.set(ControlMode.PercentOutput, leftPow);
            mlDriveX.set(ControlMode.PercentOutput, leftPow);
            blDriveX.set(ControlMode.PercentOutput, leftPow);
            frDriveX.set(ControlMode.PercentOutput, rightPow);
            mrDriveX.set(ControlMode.PercentOutput, rightPow);
            brDriveX.set(ControlMode.PercentOutput, rightPow);
        } else {
            flDrive.set(leftPow);
            mlDrive.set(leftPow);
            blDrive.set(leftPow);
            frDrive.set(rightPow);
            mrDrive.set(rightPow);
            brDrive.set(rightPow);
        }

    }

    /**
     * manual operator controlled drive
     */

    public void operatorDrive() {
        double leftY = 0;
        double rightY = 0;

        if (Math.abs(Robot.leftJoystick.getY()) > deadband) {
            leftY = Math.pow(Robot.leftJoystick.getY(), 3);
        }
        if (Math.abs(Robot.rightJoystick.getY()) > deadband) {
            rightY = -Math.pow(Robot.rightJoystick.getY(), 3);
        }

        checkForGearShift();

        if (!isShifting) {
            assignMotorPower(leftY, rightY);
        } else {

            assignMotorPower(0, 0);
        }

    }

    /**
     * Checks for joystick input to shift gears. Manages to logic and timing to not
     * power drive motors while shifting
     */
    public void checkForGearShift() {
        boolean shiftHi = Robot.rightJoystick.getRawButton(HI_SHIFTER);
        boolean shiftLo = Robot.rightJoystick.getRawButton(LO_SHIFTER);

        if (shiftHi) {
            currentGear = Gear.HI;
            if (shiftInit) {
                shiftTimer.setTimer(Constants.SHIFT_TIME);
                shiftInit = false;
            }
            if (shiftTimer.isExpired()) {
                isShifting = false;
                shiftInit = true;
            } else {
                isShifting = true;
            }
            gearShifter.set(Constants.HI_GEAR_VALUE);
        } else if (shiftLo) {
            currentGear = Gear.LO;
            if (shiftInit) {
                shiftTimer.setTimer(Constants.SHIFT_TIME);
                shiftInit = false;
            }
            if (shiftTimer.isExpired()) {
                isShifting = false;
                shiftInit = true;
            } else {
                isShifting = true;
            }
            gearShifter.set(Constants.LO_GEAR_VALUE);
        } else {
            isShifting = false;
        }

    }

    /**
     * current gear status
     */

    public enum Gear {
        HI, LO
    }

    /**
     * 
     * @param shiftTo desired gear
     */
    public void shiftGear(Gear shiftTo) {
        boolean shiftHi = false;
        boolean shiftLo = false;

        currentGear = shiftTo;

        if (shiftTo == Gear.HI) {
            shiftHi = true;
        } else {
            shiftLo = true;
        }

        if (shiftHi) {
            if (shiftInit) {
                shiftTimer.setTimer(Constants.SHIFT_TIME);
                shiftInit = false;
            }
            if (shiftTimer.isExpired()) {
                isShifting = false;
                shiftInit = true;
            } else {
                isShifting = true;
            }
            gearShifter.set(Constants.HI_GEAR_VALUE);
        } else if (shiftLo) {
            if (shiftInit) {
                shiftTimer.setTimer(Constants.SHIFT_TIME);
                shiftInit = false;
            }
            if (shiftTimer.isExpired()) {
                isShifting = false;
                shiftInit = true;
            } else {
                isShifting = true;
            }
            gearShifter.set(Constants.LO_GEAR_VALUE);
        } else {
            isShifting = false;
        }

    }

    public void updateTelemetry(){
        
    }

}
