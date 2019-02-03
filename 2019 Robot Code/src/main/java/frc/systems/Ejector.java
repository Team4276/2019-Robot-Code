package frc.systems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.utilities.Xbox;
import frc.utilities.SoftwareTimer;

public class Ejector {
    private Solenoid ejectSol;
    private SoftwareTimer ejectTime;
    double activateTime = 0.25;
    boolean isEjecting = false;

    public Ejector(int port1) {
        ejectSol = new Solenoid(port1);
        ejectTime = new SoftwareTimer();
    }

    public void performMainProcessing() {
        if (Math.abs(Robot.xboxJoystick.getRawAxis(Xbox.RT)) > .2) {
            Eject();
        }
        else{
            unject();
        }
        updateTelemetry();
    }

    public void Eject() {
        isEjecting = true;
        ejectSol.set(true);
        //ejectTime.setTimer(activateTime);

    }
    public void unject() {
        isEjecting = false;
        ejectSol.set(false);

    }
    public void checkForUnject() {
        if (ejectTime.isExpired()) {
            ejectSol.set(false);
            isEjecting = false;
        }
    }

    public void updateTelemetry() {
        SmartDashboard.putBoolean("Ejecting?", isEjecting);
    }
}
