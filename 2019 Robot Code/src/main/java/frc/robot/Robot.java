/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import frc.systems.DriveSystem;
import frc.systems.Ejector;
import frc.systems.sensors.Cameras;
import frc.systems.sensors.IMU;
import frc.utilities.RoboRioPorts;
import frc.systems.ArmPivot;
import frc.systems.BallLift;
import frc.systems.Collector;
import edu.wpi.first.wpilibj.Joystick;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static Joystick leftJoystick;
  public static Joystick rightJoystick;
  public static Joystick xboxJoystick;
  public static Timer systemTimer;
  public static IMU mImu;

  Cameras robotCameraSystem;

  Notifier driveRateGroup;
  DriveSystem mDriveSystem;

  Notifier liftRateGroup;
  BallLift mBallLift;

  Notifier collectorRateGroup;
  Collector mCollector;

  Notifier ejectorRateGroup;
  Ejector mEjector;

  Notifier armRateGroup;
  ArmPivot mArmPivot;

  private double timeCurr = 0;
  private double timeLast = 0;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    systemTimer = new Timer();
    mImu = new IMU();
    robotCameraSystem = new Cameras();

    leftJoystick = new Joystick(0);
    rightJoystick = new Joystick(1);
    xboxJoystick = new Joystick(2);

    mDriveSystem = new DriveSystem(false, 1, 2, 5, 3, 4, 6, 0, 1, RoboRioPorts.DIO_DRIVE_RIGHT_A,
        RoboRioPorts.DIO_DRIVE_RIGHT_B, RoboRioPorts.DIO_DRIVE_LEFT_A, RoboRioPorts.DIO_DRIVE_LEFT_B);

    mBallLift = new BallLift(8, 9, RoboRioPorts.DIVERTER_FWD, RoboRioPorts.DIVERTER_REV);

    mCollector = new Collector(7);

    mEjector = new Ejector(RoboRioPorts.EJECTOR_PISTON_FWD, RoboRioPorts.EJECTOR_PISTON_REV);

    mArmPivot = new ArmPivot(RoboRioPorts.CAN_ARM_PIVOT1, RoboRioPorts.CAN_ARM_PIVOT2, RoboRioPorts.DIO_ARM_A,
        RoboRioPorts.DIO_ARM_B, RoboRioPorts.ARM_LIM_SWITCH);

    driveRateGroup = new Notifier(mDriveSystem::operatorDrive);
    liftRateGroup = new Notifier(mBallLift::performMainProcessing);
    collectorRateGroup = new Notifier(mCollector::performMainProcessing);
    ejectorRateGroup = new Notifier(mEjector::performMainProcessing);
    armRateGroup = new Notifier(mArmPivot::performMainProcessing);

    armRateGroup.startPeriodic(.025);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    //mDriveSystem.drivePath("MidToFrontLeft");
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  /**
   * 
   */
  @Override
  public void teleopInit() {

    driveRateGroup.startPeriodic(0.05);
    liftRateGroup.startPeriodic(0.2);
    collectorRateGroup.startPeriodic(0.2);
    ejectorRateGroup.startPeriodic(0.2);
    super.teleopInit();
  }

  /**
   * 
   */
  @Override
  public void disabledInit() {
    ejectorRateGroup.stop();
    collectorRateGroup.stop();
    liftRateGroup.stop();
    driveRateGroup.stop();
    super.disabledInit();
  }

  /**
   * 
   */
  @Override
  public void disabledPeriodic() {
    mDriveSystem.updateTelemetry();
    mArmPivot.updateTelemetry();
    mBallLift.updateTelemetry();
    mCollector.updateTelemetry();
    mEjector.updateTelemetry();

    super.disabledPeriodic();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
