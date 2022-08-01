package frc.robot.subsystems;

import SushiFrcLib.Motor.MotorHelper;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import libraries.cheesylib.loops.Loop.Phase;
import libraries.cheesylib.subsystems.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.constants.StaticStates.ShooterState;
import frc.robot.constants.Constants;
import frc.robot.constants.Ports;

public class Shooter extends Subsystem<ShooterState> {

    //Hardware
    private final WPI_TalonFX backRoller;
    // private final WPI_TalonFX frontMain;
    // private final WPI_TalonFX frontFollower;

    //Subsystem Constants
    private static final int TARMAC_RPM = 5000;
    private static final double TARMAC_RATIO = 0.7;
    private static final int FENDER_RPM = 3000;
    private static final double FENDER_RATIO = 1;
    private final int kSchedDeltaDormant = 100;
    private final int kSchedDeltaActive = 5;

    //Subsystem Variables
    public static class mPeriodicIO {
        //Inputs
        public static double frontSpeed;
        public static double backSpeed;
    }

    //Subsystem Creation
    private static Shooter sInstance = null;

    public static Shooter getInstance(String caller) {
        if (sInstance == null) {
            sInstance = new Shooter(caller);
        } else {
            sInstance.printUsage(caller);
        }
        return sInstance;
    }

    private Shooter(String caller) {
        backRoller = MotorHelper.createFalconMotor(Ports.BACK_ROLLER_ID, Constants.FALCON_CURRENT_LIMIT, TalonFXInvertType.Clockwise);
        printUsage(caller);
    }

    @Override
    public void start(Phase phase) {    
        synchronized (Shooter.this) {
            setState(ShooterState.DISABLED);
            setWantedState(ShooterState.DISABLED, "Shooter");
            setPeriod(kSchedDeltaDormant);
        }
    }

    @Override
    public void changeState() {
        switch(getCurrentState()) {
            case TARMAC:
                tarmacShot();
                break;
            case FENDER:
                fenderShot();
                break;
            case DISABLED:
                handleDisable();
                break;
        }       
    }

    private void tarmacShot() {
        if(stateChanged()) {
            setPeriod(kSchedDeltaActive);
            mPeriodicIO.frontSpeed = TARMAC_RPM * TARMAC_RATIO;
            mPeriodicIO.backSpeed = TARMAC_RPM;
        }
    }

    private void fenderShot() {
        if( stateChanged() ) {
            setPeriod(kSchedDeltaActive);
            mPeriodicIO.frontSpeed = FENDER_RPM * FENDER_RATIO;
            mPeriodicIO.backSpeed = FENDER_RPM;
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
        mPeriodicIO.frontSpeed = 0;
        mPeriodicIO.backSpeed = 0; 
    }

    @Override
    public void readPeriodic() {

    }

    @Override
    public void writePeriodic() {
        backRoller.set(ControlMode.PercentOutput, mPeriodicIO.backSpeed/6000.0);
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
        SmartDashboard.putNumber("back speed", mPeriodicIO.backSpeed);    
    }

}
