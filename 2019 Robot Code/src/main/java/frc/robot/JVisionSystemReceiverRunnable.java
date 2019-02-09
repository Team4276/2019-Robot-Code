package org.usfirst.frc.team4276.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author viking
 */
public class JVisionSystemReceiverRunnable implements Runnable 
{
    boolean m_continueRunning;

    JReceiver m_visionSystemReceiver;
    JTargetInfo m_visionSystemTargetInfo;

    @Override
    public void run() {
        m_visionSystemReceiver = new JReceiver();
        m_visionSystemTargetInfo = new JTargetInfo();
        m_visionSystemReceiver.init();
  
        SmartDashboard.putNumber("visionSystemPixelX",Robot.g_visionSystemPixelX);                

        String textInput;
        m_continueRunning = true;
        while(m_continueRunning) 
        {
            textInput = m_visionSystemReceiver.getOneLineFromSocket();
            if(textInput != null)
            {
                Robot.g_nSequenceVisionSystem++;
                SmartDashboard.putNumber("visionSystemPixelX",Robot.g_visionSystemPixelX);                
                //System.out.println(textInput);
                Robot.g_isVisionSystemGoalDetected = m_visionSystemTargetInfo.m_isUpperGoalFound;
                Robot.g_visionSystemAngleRobotToGoal = m_visionSystemTargetInfo.m_angleFromStraightAheadToUpperGoal;
                Robot.g_visionSystemPixelX = m_visionSystemTargetInfo.m_pixelX;
                
                SmartDashboard.putBoolean("isVisionSystemGoalDetected",Robot.g_isVisionSystemGoalDetected); 
                SmartDashboard.putNumber("visionSystemAngleRobotToGoal",Robot.g_visionSystemAngleRobotToGoal);                
                SmartDashboard.putNumber("visionSystemPixelX",Robot.g_visionSystemPixelX);                
            }
            else
            {
                Robot.g_isVisionSystemGoalDetected = false;
            }
        }
    }
}