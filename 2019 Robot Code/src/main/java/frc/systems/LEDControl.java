/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.systems;

import edu.wpi.first.wpilibj.DigitalOutput;

/**
 * Add your docs here.
 */
public class LEDControl {

    DigitalOutput enablePin;
    DigitalOutput intakePin;
    DigitalOutput ballPin;
    DigitalOutput shootingPin;
    private static LightMode currentMode = LightMode.DISABLE;

    public LEDControl(int ePort, int iPort, int bPort, int sPort) {

        enablePin = new DigitalOutput(ePort);
        intakePin = new DigitalOutput(ePort);
        ballPin = new DigitalOutput(ePort);
        shootingPin = new DigitalOutput(ePort);

    }

    public static enum LightMode {
        DISABLE, ENABLE, INTAKE, FULL, SHOOT
    }

    public void setMode(LightMode mode) {
        currentMode = mode;
    }

    public void performMainProcessing() {
        switch (currentMode) {
        case DISABLE:
            enablePin.set(false);
            intakePin.set(false);
            ballPin.set(false);
            shootingPin.set(false);
            break;
        case ENABLE:
            enablePin.set(true);
            intakePin.set(false);
            ballPin.set(false);
            shootingPin.set(false);
            break;
        case INTAKE:
            enablePin.set(true);
            intakePin.set(true);
            ballPin.set(false);
            shootingPin.set(false);
            break;
        case FULL:
            enablePin.set(true);
            intakePin.set(false);
            ballPin.set(true);
            shootingPin.set(false);
            break;
        case SHOOT:
            enablePin.set(true);
            intakePin.set(false);
            ballPin.set(false);
            shootingPin.set(true);
            break;
        default:
            break;
        }
    }
}
