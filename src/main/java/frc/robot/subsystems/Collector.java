package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.constants.Constants;
import frc.robot.constants.Ports;
import libraries.cheesylib.drivers.TalonFXFactory;
import libraries.cheesylib.loops.Loop.Phase;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cheesylib.util.LatchedBoolean;
import libraries.cyberlib.control.FramePeriodSwitch;
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

    public static class mPeriodicIO {
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
    private static Collector sInstance = null;

    public static Collector getInstance(String caller) {
        if (sInstance == null) {
            sInstance = new Collector(caller);
        } else {
            sInstance.printUsage(caller);
        }
        return sInstance;
    }

    private Collector(String caller) {
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
    }

    @Override
    public void start(Phase phase) {
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
            setPeriod(kSchedDeltaActive);
        }
    }

    @Override
    public void onLoop() {
        synchronized (Collector.this) {
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
        }
    }

    public boolean isHandlerComplete(CollecterState state) {
        switch(state) {
            case ASSESSING:
                return mAssessmentHandlerComplete;
            default:
                System.out.println("Uh oh something is not right in "+subsystemName);
                return false;
        }
    }

    private void handleAssessing() {
        if (stateChanged()) {
            mAssessingStartPosition = mPeriodicIO.motorPosition;
            setSolenoidDemand(SolenoidState.RETRACT);
            setMotorControlModeAndDemand(ControlMode.PercentOutput,kAssessingMotorDemand);
            setPeriod(kSchedDeltaActive);
            mAssessmentResult = false;
            mAssessmentHandlerComplete = false;
            mHandlerLoopCount = kMotorAssessTime / kSchedDeltaActive; // 250 is .25 sec to run collector forward
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
        if (stateChanged()) {
            setSolenoidDemand(SolenoidState.EXTEND);
            setMotorControlModeAndDemand(ControlMode.PercentOutput,kCollectSpeed);
            setPeriod(kSchedDeltaActive);
            // need to run motor forward until collector extends beyond bumper
            // before reversing for backing
            mHandlerLoopCount = (kBackingEjectDuration / (double) kSchedDeltaActive); 
            mLB_handlerLoopCounter.update(false); // reset
        }

        // Run collector forward to deploy collector before backing
        if(mLB_handlerLoopCounter.update(mHandlerLoopCount-- <= 0)) {
            setMotorControlModeAndDemand(ControlMode.PercentOutput,-kCollectSpeed);
        }
    }

    private void handleCollecting() {
        if(stateChanged()) {
            setSolenoidDemand(SolenoidState.EXTEND);
            setMotorControlModeAndDemand(ControlMode.PercentOutput,kCollectSpeed);
            setPeriod(kSchedDeltaActive);
        }
    }

    private void handleFeeding() {
        if(stateChanged()) {
            setMotorControlModeAndDemand(ControlMode.PercentOutput,kFeedSpeed);
            setPeriod(kSchedDeltaActive);
        }
    }


    private void handleDisabling() {
        if (stateChanged()) {
            setSolenoidDemand(SolenoidState.RETRACT);
            setMotorControlModeAndDemand(ControlMode.PercentOutput,0.0);
            setPeriod(kSchedDeltaDormant);
        }
    }

    private void handleHolding() {
        if (stateChanged()) {
            setSolenoidDemand(SolenoidState.RETRACT);
            setMotorControlModeAndDemand(ControlMode.PercentOutput,0.0);
            setPeriod(kSchedDeltaDormant);
        }
    }

    private void handleManualControlling() {
        if (stateChanged()) {
            mTestMotorDemand = 0;
            mTestSolenoidDemand = SolenoidState.RETRACT;
            setPeriod(kSchedDeltaActive);
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
    public void readPeriodic() {
        mPeriodicIO.motorPosition = FramePeriodSwitch.getSelectedSensorPosition(mFXMotor);
    }

    @Override
    public void writePeriodic() {
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
    }

    @Override
    public String getPeriodicLogHeaders() {
        return subsystemName+".motorControlMode,"+
                subsystemName+".motorDemand,"+
                subsystemName+".motorPosition,"+
                subsystemName+".motorStator,"+
                subsystemName+".solenoidState";
    }

    @Override
    public String getLogValues() {
        return  mPeriodicIO.motorControlMode+","+
                mPeriodicIO.motorDemand+","+
                mPeriodicIO.motorPosition+","+
                mPeriodicIO.motorStator+","+
                mPeriodicIO.solenoidState;
    }

    @Override
    public void outputTelemetry() {
        // mPeriodicIO.motorStator = FramePeriodSwitch.getStatorCurrent(mFXMotor);
    }
}