/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.systems.sensors;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Limelight {

   public boolean LimelightHasValidTarget = false;
   public  double rightSteering = 0.0;
   public  double leftSteering = 0.0;

   double distance = 0.0;

  //steering commands
  final double Kp = -0.05;                    
  final double minSteer = 0.05;
  //driving commands
  final double DRIVE_K = 0.26;                    
  final double DESIRED_TARGET_AREA = 13.0;        
  final double MAX_DRIVE = 0.7;                   

  public double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  public double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  public double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  public double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  public double ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);

  public Limelight(){
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
  }

  

  public void RotateTracking()
  {
    setLedMode(3);
    
    setCamMode(0);

        if (tv < 1.0)
        {
          LimelightHasValidTarget = false;
          rightSteering = 0.0;
          leftSteering = 0.0;
          return;
        }

    
        LimelightHasValidTarget = true;

        double steerError = tx;
        double steerCmd = 0;
/*
        if(tx > 1.0){
          steerCmd = Kp*steerError - minSteer;
        }
        else if(tx < 1.0){
          steerCmd = Kp*steerError + minSteer;
        }*/
        steerCmd = Kp*tx;

        rightSteering -= steerCmd;
        leftSteering += steerCmd;
        
      
      //setCamMode(1);
      //setLedMode(1);
    }

public void DriveTracking(){

}

public void setLedMode(int mode){
  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(mode);
}

public void setCamMode(int mode){
  NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(mode);
}

public void setPipeline(int mode){
  NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(mode);
}

public double getDistance(){
  return distance;
}

public double getSteer(double steer){
  return steer;
}

public double getTx(){
  return tx;
}

public void updateTelementry(){
  tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
  tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
  ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
  ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
  ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);

  SmartDashboard.putBoolean("Has Target", LimelightHasValidTarget);
    SmartDashboard.putNumber("KP Steering", Kp);
    SmartDashboard.putNumber("ts", ts);
    SmartDashboard.putNumber("tv", tv);
    SmartDashboard.putNumber("tx", tx);
}

}