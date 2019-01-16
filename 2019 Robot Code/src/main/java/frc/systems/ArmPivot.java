/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.utilities.Constants;

/**
 * Add your docs here.
 */
public class ArmPivot {

    TalonSRX pivotMotor;

    public ArmPivot(int pivotMotorPort) {

        pivotMotor = new TalonSRX(pivotMotorPort);

        /* Disable motors */
        pivotMotor.set(ControlMode.PercentOutput, 0);

        /* Factory Default all hardware to prevent unexpected behavior */
        pivotMotor.configFactoryDefault();

        /* Set neutral modes */
        pivotMotor.setNeutralMode(NeutralMode.Brake);

        /* Configure sensor **SENSOR SHOULD BE ADJUSTED** */
        pivotMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.PID_PRIMARY, Constants.CAN_TimeoutMs);
    
        /* configure inversion **ADJUST** */
        pivotMotor.setInverted(true);
    }
}
