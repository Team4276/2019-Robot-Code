/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.utilities;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * Add your docs here.
 */
public class Constants {

//for drive shifting gearbox

public final static double SHIFT_TIME = 0.05; // sec

public final static Value HI_GEAR_VALUE = DoubleSolenoid.Value.kForward;
public final static Value LO_GEAR_VALUE = DoubleSolenoid.Value.kReverse;

    /**
     * Set to zero to skip waiting for confirmation. Set to nonzero to wait and
     * report to DS if action fails.
     */
    public final static int CAN_TimeoutMs = 30;

    public final static int PID_PRIMARY = 0;

}
