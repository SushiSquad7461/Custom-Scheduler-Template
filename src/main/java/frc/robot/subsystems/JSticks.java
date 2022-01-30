package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import libraries.cheesylib.loops.ILooper;
import libraries.cheesylib.loops.Loop;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.cyberlib.io.CW;
import libraries.cyberlib.io.Xbox;

public class JSticks extends Subsystem{

    public enum SystemState {
        READINGBUTTONS,
    }

    public enum WantedState {
        READBUTTONS,
    }

    private SystemState mSystemState = SystemState.READINGBUTTONS;
    private WantedState mWantedState = WantedState.READBUTTONS;
    @SuppressWarnings("unused")
    private boolean mStateChanged;
    private CW mDriver;
    // private CW mOperator;
    private final double mDeadBand = 0.15; // for the turnigy (driver) swerve controls
	// private Superstructure mSuperstructure;
    private Swerve mSwerve;

    //Logging
    private final int mDefaultSchedDelta = 100; // axis updated every 100 msec
    public  int    schedDeltaDesired;
    public  double schedDeltaActual;
    public  double schedDuration;
    private double lastSchedStart;

    //Joystick Inputs
    public double  dr_RightStickX_Translate; // drive
    public double  dr_RightStickY_Translate; // drive
    public double  dr_LeftStickX_Rotate;     // drive
    public boolean dr_YButton_ResetIMU = false;      // reset direction
    public boolean dr_LeftToggleDown_RobotOrient = false; // field/robot oriented

    private static String sClassName;
    private static int sInstanceCount;
    private static JSticks sInstance = null;
    public  static JSticks getInstance(String caller) {
        if(sInstance == null) {
            sInstance = new JSticks(caller);
        }
        else {
            printUsage(caller);
        }
        return sInstance;
    }

    private static void printUsage(String caller){
        System.out.println("("+caller+") "+" getInstance " + sClassName + " " + ++sInstanceCount);
    }

    private JSticks(String caller){
        sClassName = this.getClass().getSimpleName();
        // mSuperstructure = Superstructure.getInstance(sClassName);
        mSwerve = Swerve.getInstance(sClassName);
        mDriver = new Xbox();
        // mOperator = new Xbox();
        printUsage(caller);
    }

    private Loop mLoop = new Loop(){
        
        @Override
        public void onStart(Phase phase){
            synchronized (JSticks.this) {
                mSystemState = SystemState.READINGBUTTONS;
                mWantedState = WantedState.READBUTTONS;
                mStateChanged = true;
                System.out.println(sClassName + " state " + mSystemState);
                switch (phase) {
                    case DISABLED:
                    case AUTONOMOUS: 
                        schedDeltaDesired = 0; // goto sleep
                        break;
                    default:
                        schedDeltaDesired = mDefaultSchedDelta;
                        break;
                }
            }
        }

        @Override
        public void onLoop(double timestamp){
            synchronized (JSticks.this) {
                SystemState newState;
                switch (mSystemState) {
                case READINGBUTTONS:
                default:
                    newState = handleReadingButtons();
                    break;
                }

                if (newState != mSystemState) {
                    System.out.println(sClassName + " state " + mSystemState + " to " + newState + " (" + timestamp + ")");
                    mSystemState = newState;
                    mStateChanged = true;
                } else {
                    mStateChanged = false;
                }
            }
        }

        @Override
        public void onStop(double timestamp){
            stop();
        }

    };

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        
    }

    private SystemState handleReadingButtons() {
        teleopRoutines();
        
        return defaultStateTransfer();
    }

    public void teleopRoutines() {
        //Swerve control
		double swerveYInput = dr_RightStickX_Translate;
		double swerveXInput = dr_RightStickY_Translate;
		double swerveRotationInput = dr_LeftStickX_Rotate;

        mSwerve.sendInput(swerveXInput, swerveYInput, swerveRotationInput, dr_LeftToggleDown_RobotOrient, false);

		if (dr_YButton_ResetIMU) {
			mSwerve.temporarilyDisableHeadingController();
			mSwerve.zeroSensors(Constants.kRobotStartingPose);
			mSwerve.resetAveragedDirection();
		}

	}

    @Override
    public void readPeriodicInputs() {
        double now       = Timer.getFPGATimestamp();
        schedDeltaActual = now - lastSchedStart;
        lastSchedStart   = now;

        dr_RightStickX_Translate = mDriver.getRaw(Xbox.RIGHT_STICK_X, mDeadBand);
        dr_RightStickY_Translate = mDriver.getRaw(Xbox.RIGHT_STICK_Y, mDeadBand);
        dr_LeftStickX_Rotate = mDriver.getRaw(Xbox.LEFT_STICK_X, mDeadBand);
        dr_YButton_ResetIMU = mDriver.getButton(Xbox.Y_BUTTON, CW.PRESSED_EDGE);

    }

    private SystemState defaultStateTransfer() {
        switch (mWantedState) {
        case READBUTTONS:
        default:
            return SystemState.READINGBUTTONS;
        }
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(mLoop);
    }

    @Override
    public String getLogHeaders() {
        // TODO Auto-generated method stub
        return "Jsticks";
    }

    @Override
    public String getLogValues(boolean telemetry) {
        // TODO Auto-generated method stub
        return "Jsticks.Values";
    }

    @Override
    public void outputTelemetry() {
        // TODO Auto-generated method stub
        
    }   

    @Override
    public int whenRunAgain () {
        return schedDeltaDesired;
    }
}