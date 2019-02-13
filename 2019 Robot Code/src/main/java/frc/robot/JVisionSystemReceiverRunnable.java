package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author viking
 */
public class JVisionSystemReceiverRunnable implements Runnable 
{
    boolean m_continueRunning;

    JReceiver m_visionSystemReceiver;

    @Override
    public void run() {
        m_visionSystemReceiver = new JReceiver();
        m_visionSystemReceiver.init();
  
        String textInput;
        m_continueRunning = true;
        while(m_continueRunning) 
        {
            textInput = m_visionSystemReceiver.getOneLineFromSocket();
            if(textInput != null)
            {
                Robot.nSequenceVisionSystem++;
                SmartDashboard.putNumber("visionSystemPixelX",Robot.visionTargetInfo.visionPixelX);                
                Robot.visionTargetInfo.initTargetInfoFromText(textInput);
                
                SmartDashboard.putBoolean("isVisionSystemGoalDetected", Robot.visionTargetInfo.isCargoBayDetected); 
            }
            else
            {
                Robot.visionTargetInfo.isCargoBayDetected = false;
            }
        }
    }
}