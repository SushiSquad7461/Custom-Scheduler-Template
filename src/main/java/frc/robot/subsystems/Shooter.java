package frc.robot.subsystems;

import SushiFrcLib.Motor.MotorHelper;
import libraries.cheesylib.loops.Loop.Phase;
import libraries.cheesylib.subsystems.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import frc.robot.constants.StaticStates.ShooterState;
import frc.robot.constants.Constants.kShooter;
import frc.robot.constants.Ports;

public class Shooter extends Subsystem<ShooterState> {

    //Hardware
    private final WPI_TalonFX backRoller;
    // private final WPI_TalonFX frontMain;
    // private final WPI_TalonFX frontFollower;

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
        backRoller = MotorHelper.createFalconMotor(Ports.BACK_ROLLER_ID, kShooter.CURRENT_LIMIT, TalonFXInvertType.Clockwise);
        printUsage(caller);
    }

    @Override
    public void start(Phase phase) {    
        synchronized (Shooter.this) {
            setState(ShooterState.DISABLED);
            setPeriod(kShooter.kSchedDeltaDormant);
        }
    }

    @Override
    public void onLoop() {
        synchronized (Shooter.this) {
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
    }

    private void tarmacShot() {
        if(stateChanged()) {
            setPeriod(kShooter.kSchedDeltaActive);
            mPeriodicIO.frontSpeed = kShooter.TARMAC_RPM * kShooter.TARMAC_RATIO;
            mPeriodicIO.backSpeed = kShooter.TARMAC_RPM;
        }
    }

    private void fenderShot() {
        if( stateChanged() ) {
            setPeriod(kShooter.kSchedDeltaActive);
            mPeriodicIO.frontSpeed = kShooter.FENDER_RPM * kShooter.FENDER_RATIO;
            mPeriodicIO.backSpeed = kShooter.FENDER_RPM;
        }
    }

    private void handleDisable() {
        if(stateChanged()) {
            stop();
            setPeriod(kShooter.kSchedDeltaDormant);
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
        backRoller.set(ControlMode.PercentOutput, 6000.0/mPeriodicIO.backSpeed);
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
