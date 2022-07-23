package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import frc.robot.constants.Ports;
import libraries.cheesylib.drivers.TalonFXFactory;
import libraries.cheesylib.loops.Loop.Phase;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.subsystems.SubsystemManager;
import libraries.cheesylib.util.LatchedBoolean;
import libraries.cyberlib.control.FramePeriodSwitch;
import libraries.newAdditions.State;
import libraries.newAdditions.StaticStates.*;

public class Collector extends Subsystem<CollecterState> {

    // Hardware
    private final TalonFX mFXMotor;
    private final Solenoid mSolenoid;

    // Subsystem Constants
    private final double kCollectSpeed = 0.9;
    private final double kFeedSpeed = 0;
    // time (msec) to run collector motor forward so collector can deploy
    // when backing
    private final double kBackingEjectDuration = 100; 
    private final double kMotorAssessTime = 250; // quarter second
    private final double kCurrentLimit = 60;
    private final double kAssessingMotorDemand = .5;
    private final int kSchedDeltaActive = 20;
    private final int kSchedDeltaDormant = 100;
    private final double kMinAssessmentMovement = 100; // ticks

    // subsystem variables
    private double mAssessingStartPosition;
    private boolean mAssessmentResult;
    private boolean mAssessmentHandlerComplete;
    private SolenoidState mTestSolenoidDemand;
    private double mTestMotorDemand;
    private double mHandlerLoopCount;

    // latched booleans
    private LatchedBoolean mLB_handlerLoopCounter = new LatchedBoolean();

    private boolean mStateChanged;
    private LatchedBoolean mLB_SystemStateChange = new LatchedBoolean();

    public static class mPeriodicIO {
        // Logging
        public static int schedDeltaDesired;
        public static double schedDeltaActual;
        public static double lastSchedStart;

        // Inputs
        public static double motorPosition;
        public static double motorStator;

        // Outputs
        private static ControlMode motorControlMode;
        private static double motorDemand;
        private static SolenoidState solenoidDemand;

        // other
        private static SolenoidState solenoidState;
    }

    // Subsystem Creation
    private static String sClassName;
    private static int sInstanceCount;
    private static Collector sInstance = null;

    public static Collector getInstance(String caller) {
        if (sInstance == null) {
            sInstance = new Collector(caller);
        } else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller) {
        System.out.println("(" + caller + ") " + " getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private Collector(String caller) {
        super("Collector");
        sClassName = this.getClass().getSimpleName();
        printUsage(caller);
        mFXMotor = TalonFXFactory.createDefaultTalon(Ports.COLLECTOR, Constants.kCanivoreName);
        mSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Ports.COLLECTOR_DEPLOY);
        configMotors();
    }

    private void configMotors() {
        // must be run on every powerup
        FramePeriodSwitch.setFramePeriodsVolatile(mFXMotor); // set frame periods


        // The following commands are stored in nonVolatile ram in the motor
        // They are repeated on boot incase a motor needs to replaced quickly
        FramePeriodSwitch.configFactoryDefaultPermanent(mFXMotor);
        FramePeriodSwitch.configStatorCurrentLimitPermanent(mFXMotor, new StatorCurrentLimitConfiguration(true, kCurrentLimit, kCurrentLimit, 0));
    
        // the following commands are stored in nonVolatile ram but they are
        // no longer deemed necessary. Keeping around for a while in case they
        // need to be brought back
        // talon.configNeutralDeadband(config.NEUTRAL_DEADBAND, kTimeoutMs);
        // talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, kTimeoutMs);
        // talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.Disabled, kTimeoutMs);    
    
    }

    @Override
    public void onStart(Phase phase) {
        synchronized (Collector.this) { // TODO Check if the key word synchronized is needed
            switch (phase){
                case TELEOP:
                case AUTONOMOUS:
                    setState(CollecterState.HOLDING);
                    break;
                case DISABLED:
                    setState(CollecterState.DISABLING);
                    break;
                case TEST:
                    setState(CollecterState.MANUAL_CONTROLLING);
                    break;
            }
            mStateChanged = true;
            System.out.println(sClassName + " state " + getCurrentState());
            mPeriodicIO.schedDeltaDesired = kSchedDeltaActive;
            mLB_SystemStateChange.update(false); // reset
            stop(); // put into a known state
        }
    }

    @Override
    public void onLoop(double timestamp) {
        synchronized (Collector.this) {
            do {
                switch (getCurrentState()) {
                    case ASSESSING:
                        handleAssessing();
                        break;
                    case BACKING:
                        handleBacking();
                        break;
                    case COLLECTING:
                        handleCollecting();
                        break;
                    case FEEDING:
                        handleFeeding();
                        break;
                    case DISABLING:
                        handleDisabling();
                        break;
                    case HOLDING:
                        handleHolding();
                        break;
                    case MANUAL_CONTROLLING:
                        handleManualControlling();
                        break;
                    // default:
                    // leave commented so compiler will identify missing cases
                }

                CollecterState newState = getWantedState();

                if (newState != getCurrentState()) {
                    System.out.println(
                            sClassName + " state " + getCurrentState() + " to " + newState + " (" + timestamp + ")");
                    setState(newState);;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            } while (mLB_SystemStateChange.update(mStateChanged));
        }
    }

    public boolean isHandlerComplete(CollecterState state) {
        switch(state) {
            case ASSESSING:
                return mAssessmentHandlerComplete;
            default:
                System.out.println("Uh oh something is not right in "+sClassName);
                return false;
        }
    }

    private void handleAssessing() {
        if (mStateChanged) {
            mAssessingStartPosition = mPeriodicIO.motorPosition;
            setSolenoidDemand(SolenoidState.RETRACT);
            setMotorControlModeAndDemand(ControlMode.PercentOutput,kAssessingMotorDemand);
            mPeriodicIO.schedDeltaDesired = kSchedDeltaActive;
            mAssessmentResult = false;
            mAssessmentHandlerComplete = false;
            mHandlerLoopCount = kMotorAssessTime / mPeriodicIO.schedDeltaDesired; // 250 is .25 sec to run collector forward
            mLB_handlerLoopCounter.update(false); // reset
        }

        // Run collector forward to deploy collector before backing
        if(mLB_handlerLoopCounter.update(mHandlerLoopCount-- <= 0)) {
            setMotorControlModeAndDemand(ControlMode.PercentOutput,0);
            if (mAssessingStartPosition >= mPeriodicIO.motorPosition + kMinAssessmentMovement){
                mAssessmentResult = true;
                System.out.println("ASSESSING: Collector motor functioning");
            }
            else{
                System.out.println("ASSESSING: Collector motor DID NOT DETECT MOVEMENT");
            }
            mAssessmentHandlerComplete = true;
        }

        if (getWantedState() != CollecterState.ASSESSING) {
            mAssessmentHandlerComplete = false;
        }

    }

    private void handleBacking() {
        if (mStateChanged) {
            setSolenoidDemand(SolenoidState.EXTEND);
            setMotorControlModeAndDemand(ControlMode.PercentOutput,kCollectSpeed);
            mPeriodicIO.schedDeltaDesired = kSchedDeltaActive;
            // need to run motor forward until collector extends beyond bumper
            // before reversing for backing
            mHandlerLoopCount = (kBackingEjectDuration / (double) mPeriodicIO.schedDeltaDesired); 
            mLB_handlerLoopCounter.update(false); // reset
        }

        // Run collector forward to deploy collector before backing
        if(mLB_handlerLoopCounter.update(mHandlerLoopCount-- <= 0)) {
            setMotorControlModeAndDemand(ControlMode.PercentOutput,-kCollectSpeed);
        }
    }

    private void handleCollecting() {
        if(mStateChanged) {
            setSolenoidDemand(SolenoidState.EXTEND);
            setMotorControlModeAndDemand(ControlMode.PercentOutput,kCollectSpeed);
            mPeriodicIO.schedDeltaDesired = kSchedDeltaActive;
        }
    }

    private void handleFeeding() {
        if(mStateChanged) {
            setMotorControlModeAndDemand(ControlMode.PercentOutput,kFeedSpeed);
            mPeriodicIO.schedDeltaDesired = kSchedDeltaActive;
        }
    }


    private void handleDisabling() {
        if (mStateChanged) {
            setSolenoidDemand(SolenoidState.RETRACT);
            setMotorControlModeAndDemand(ControlMode.PercentOutput,0.0);
            mPeriodicIO.schedDeltaDesired = kSchedDeltaDormant;
        }
    }

    private void handleHolding() {
        if (mStateChanged) {
            setSolenoidDemand(SolenoidState.RETRACT);
            setMotorControlModeAndDemand(ControlMode.PercentOutput,0.0);
            mPeriodicIO.schedDeltaDesired = kSchedDeltaDormant;
        }
    }

    private void handleManualControlling() {
        if (mStateChanged) {
            mTestMotorDemand = 0;
            mTestSolenoidDemand = SolenoidState.RETRACT;
            mPeriodicIO.schedDeltaDesired = kSchedDeltaActive;
        }

        setMotorControlModeAndDemand(ControlMode.PercentOutput, mTestMotorDemand);
        setSolenoidDemand(mTestSolenoidDemand);
    }

    public boolean getLastAssessmentResult(){
        return mAssessmentResult;
    }

    public void setMotorTestDemand(double newDemand){
        mTestMotorDemand = newDemand;
    }

    public void setSolenoidDemand(boolean extend){
        if (extend){
            mTestSolenoidDemand = SolenoidState.EXTEND;
        }
        else{
            mTestSolenoidDemand = SolenoidState.RETRACT;
        }
    }

    private void setMotorControlModeAndDemand(ControlMode controlMode, double demand){
        mPeriodicIO.motorControlMode = controlMode;
        mPeriodicIO.motorDemand = demand;
    }

    private void setSolenoidDemand(SolenoidState newSolenoidState){
        mPeriodicIO.solenoidDemand = newSolenoidState;
    }

    @Override
    public void readPeriodicInputs() {
        double now = Timer.getFPGATimestamp();
        mPeriodicIO.schedDeltaActual = now - mPeriodicIO.lastSchedStart;
        mPeriodicIO.lastSchedStart = now;

        mPeriodicIO.motorPosition = FramePeriodSwitch.getSelectedSensorPosition(mFXMotor);

    }

    @Override
    public void writePeriodicOutputs() {
        if (mPeriodicIO.solenoidState != mPeriodicIO.solenoidDemand) {
            mPeriodicIO.solenoidState = mPeriodicIO.solenoidDemand;
            mSolenoid.set(mPeriodicIO.solenoidDemand.get());
        }
        mFXMotor.set(mPeriodicIO.motorControlMode, mPeriodicIO.motorDemand);
    }

    @Override
    public void stop() {
        setSolenoidDemand(SolenoidState.RETRACT);

        setMotorControlModeAndDemand(ControlMode.PercentOutput,0);
        writePeriodicOutputs();
    }

    @Override
    public int whenRunAgain() {
        return mPeriodicIO.schedDeltaDesired;
    }

    @Override
    public String getLogHeaders() {
        return  sClassName+".schedDeltaDesired,"+
                sClassName+".schedDeltaActual,"+
                sClassName+".schedDuration,"+
                sClassName+".mSystemState,"+
                sClassName+".motorControlMode,"+
                sClassName+".motorDemand,"+
                sClassName+".motorPosition,"+
                sClassName+".motorStator,"+
                sClassName+".solenoidState";
    }

    @Override
    public String getLogValues(boolean telemetry) {
        String start;
        if (telemetry){
            start = ",,,";
        }
        else{
            start = mPeriodicIO.schedDeltaDesired+","+
                    mPeriodicIO.schedDeltaActual+","+
                    (Timer.getFPGATimestamp()-mPeriodicIO.lastSchedStart)+",";
        }
        return  start+
                getCurrentState() +","+
                mPeriodicIO.motorControlMode+","+
                mPeriodicIO.motorDemand+","+
                mPeriodicIO.motorPosition+","+
                mPeriodicIO.motorStator+","+
                mPeriodicIO.solenoidState;
    }

    @Override
    public void outputTelemetry() {
        mPeriodicIO.motorStator = FramePeriodSwitch.getStatorCurrent(mFXMotor);
    }
}