/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.systems.sensors;

import com.analog.adis16448.frc.ADIS16448_IMU;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class IMU {

    ADIS16448_IMU imu;

    public IMU (){
        imu = new ADIS16448_IMU();
        imu.reset();
        imu.calibrate();
    }

    public double getYaw(){
        return imu.getYaw();
    }

    public void giveReadouts(){ 
        SmartDashboard.putNumber("Gyro-X", imu.getAngleX());
        SmartDashboard.putNumber("Gyro-Y", imu.getAngleY());
        SmartDashboard.putNumber("Gyro-Z", imu.getAngleZ());
        
        SmartDashboard.putNumber("Accel-X", imu.getAccelX());
        SmartDashboard.putNumber("Accel-Y", imu.getAccelY());
        SmartDashboard.putNumber("Accel-Z", imu.getAccelZ());
        
        SmartDashboard.putNumber("Pitch", imu.getPitch());
        SmartDashboard.putNumber("Roll", imu.getRoll());
        SmartDashboard.putNumber("Yaw", imu.getYaw());
        
        SmartDashboard.putNumber("Pressure: ", imu.getBarometricPressure());
        SmartDashboard.putNumber("Temperature: ", imu.getTemperature()); 
      }
}
