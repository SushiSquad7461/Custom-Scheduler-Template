package libraries.cheesylib.subsystems;

import libraries.cheesylib.loops.ILooper;
import libraries.cheesylib.loops.Loop.Phase;

/**
 * The Subsystem abstract class, which serves as a basic framework for all robot subsystems. Each subsystem outputs
 * commands to SmartDashboard, has a stop routine (for after each match), and a routine to zero all sensors, which helps
 * with calibration.
 * <p>
 * All Subsystems only have one instance (after all, one robot does not have two drivetrains), and functions get the
 * instance of the drivetrain and act accordingly. Subsystems are also a state machine with a desired state and actual
 * state; the robot code will try to match the two states with actions. Each Subsystem also is responsible for
 * instantializing all member components at the start of the match.
 */
public abstract class Subsystem<SubsystemState extends Enum<SubsystemState>>  {
    private SubsystemManager mSubsystemManager;
    protected int mListIndex;

    private SubsystemState currentState;
    protected SubsystemState wantedState;

    private String subsystemName;

    public Subsystem(String name) {
        this.subsystemName = name;
        mSubsystemManager = SubsystemManager.getInstance(name);
    }

    // Optional design pattern for caching periodic reads to avoid hammering the HAL/CAN.
    public void readPeriodicInputs() {}

    // Optional design pattern for caching periodic writes to avoid hammering the HAL/CAN.
    public void writePeriodicOutputs() {}

    public int whenRunAgain () {return 20;}

    public void passInIndex(int listIndex){
        mListIndex = listIndex;
    }

    public void onStart(Phase phase){}

    public void onLoop(double timestamp){}

    public void zeroSensors() {}

    public abstract void stop();

    public abstract String getLogHeaders();

    public abstract String getLogValues(boolean telemetry);

    public abstract void outputTelemetry();

    public void scheduleForStateChange() {
        mSubsystemManager.scheduleMe(mListIndex, 1, true);
    }

    public synchronized SubsystemState getCurrentState() {
        return currentState;
    }

    protected SubsystemState transferState() {
        currentState = wantedState;
        return currentState;
    }

    protected void setState(SubsystemState newState) {
        currentState = newState;
    }


    // this method should only be used by external subsystems.
    // if you want to change your own wantedState then simply set
    // it directly
    public synchronized void setWantedState(SubsystemState state, String who) {
        if (state != wantedState) {
            wantedState = state;
            scheduleForStateChange();
            System.out.println(who + " is setting wanted state of " + subsystemName + " to " + state);
        } else {
            System.out.println(who + " is setting wanted state of " + subsystemName + " to " + state + " again!!!");
        }
    }

    public SubsystemState getWantedState() {
        return wantedState;
    }
}