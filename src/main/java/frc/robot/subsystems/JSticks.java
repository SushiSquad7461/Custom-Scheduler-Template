package frc.robot.subsystems;

import libraries.cheesylib.loops.Loop.Phase;
import libraries.cheesylib.subsystems.Subsystem;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.constants.StaticStates.JStickState;
import frc.robot.constants.StaticStates.ShooterState;

public class JSticks extends Subsystem<JStickState> {

    //Hardware
    private final XboxController controller;

    //Subsystem Constants
    private final int kSchedDeltaDormant = 100;
    private final int kSchedDeltaActive = 5;

    //Subsystem Variables
    public final Shooter mShooter;
    public static class mPeriodicIO {
        public static boolean reading;
    }

    //Subsystem Creation
    private static JSticks sInstance = null;

    public static JSticks getInstance(String caller) {
        if (sInstance == null) {
            sInstance = new JSticks(caller);
        } else {
            sInstance.printUsage(caller);
        }
        return sInstance;
    }

    private JSticks(String caller) {
        controller = new XboxController(0);
        mShooter = Shooter.getInstance("JSticks");
        printUsage(caller);
    }

    @Override
    public void start(Phase phase) {    
        synchronized (JSticks.this) {
            switch (phase) {
                case DISABLED:
                case AUTONOMOUS:
                    setPeriod(0);
                    setState(JStickState.DISABLED);
                    setWantedState(JStickState.DISABLED, "JSticks");
                    break;
                case TEST:
                default:
                    setState(JStickState.READING);
                    setWantedState(JStickState.READING, "JSticks");
                    setPeriod(kSchedDeltaActive);
            }
        }
    }

    @Override
    public void onLoop() {
        synchronized (JSticks.this) {
            switch(getCurrentState()) {
                case READING:
                    readButtons();
                case DISABLED:
                    handleDisable();
                    break;
            }
        }       
    }

    private void readButtons() {
        if(stateChanged()) {
            mPeriodicIO.reading = true;
        }
    }

    private void handleDisable() {
        if(stateChanged()) {
            stop();
            setPeriod(kSchedDeltaDormant);
        }
    }

    @Override
    public void stop() {
        mPeriodicIO.reading = false;
    }

    @Override
    public void readPeriodic() {
        if( controller.getYButton() ) {
            mShooter.setWantedState(ShooterState.FENDER, "JSticks");
        } else if (controller.getAButton()) {
            mShooter.setWantedState(ShooterState.TARMAC, "JSticks");
        } else {
            mShooter.setWantedState(ShooterState.DISABLED, "JSticks");
        }
    }

    @Override
    public void writePeriodic() {
    }

    @Override
    public String getPeriodicLogHeaders() {
        return "";
    }

    @Override
    public String getLogValues() {
        return "";
    }

    @Override
    public void outputTelemetry() {        
    }

}