package frc.robot.subsystems;

import SushiFrcLib.Constants.SushiConstants;
import SushiFrcLib.Scheduler.Loops.Loop.Phase;
import SushiFrcLib.Scheduler.Subsystems.Subsystem;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.constants.StaticStates.JStickState;

public class JSticks extends Subsystem<JStickState> {

    //Hardware
    private final XboxController controller;

    //Subsystem Constants
    private final int kSchedDeltaActive = 5;
    private int kSchedDeltaDormant = 100;

    //Subsystem Variables
    public final ExampleSubsystem mShooter;

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
        controller = new XboxController(SushiConstants.OI.DRIVER_PORT);
        mShooter = ExampleSubsystem.getInstance("JSticks");
        printUsage(caller);
    }

    @Override
    public void start(Phase phase) {    
        synchronized (JSticks.this) {
            switch (phase) {
                case AUTONOMOUS:
                    setPeriod(0);
                    setState(JStickState.DISABLED);
                    setWantedState(JStickState.DISABLED, "JSticks");
                    break;
                default:
                    setState(JStickState.READING);
                    setWantedState(JStickState.READING, "JSticks");
                    setPeriod(kSchedDeltaActive);
            }
        }
    }

    @Override
    public void periodic() {
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
        transferState();
    }

    private void handleDisable() {
        if(stateChanged()) {
            stop();
            setPeriod(kSchedDeltaDormant);
        }
        transferState();
    }

    @Override
    public void stop() {
    }
}