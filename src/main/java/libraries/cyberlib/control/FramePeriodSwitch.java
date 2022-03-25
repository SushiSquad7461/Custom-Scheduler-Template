package libraries.cyberlib.control;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Timer;

public class FramePeriodSwitch {
    TalonFX mFXMotor;
    int mActiveFramePeriod; 
    int mDormantFramePeriod;
    boolean mLastRunWasActive;

    final int kTimeout = 0;
    public FramePeriodSwitch(TalonFX FXMotor, int activeFramePeriod, int dormantFramePeriod){
        mFXMotor = FXMotor;
        mActiveFramePeriod = activeFramePeriod;
        mDormantFramePeriod = dormantFramePeriod;
        ErrorCode retVal;
        int retries;

        // Advanced Control frame for motor control
        // no documentation found, will keep low
        // do {
        //     retVal = mFXMotor.setControlFramePeriod(ControlFrame.Control_4_Advanced, Math.min(20,mActiveFramePeriod));
        //     System.out.println("setControlFramePeriod(ControlFrame.Control_4_Advanced) completed with "+retVal.toString());
        //     Timer.delay(.001);
        // } while (retVal != ErrorCode.OK);
        
        // General Control frame for motor control
        // must be kept low (20 msec?) for proper motor function
        retries = 5;
        do {
            retVal = mFXMotor.setControlFramePeriod(ControlFrame.Control_3_General, Math.min(20,mActiveFramePeriod));
            
            if (retVal.equals(ErrorCode.OK)){
                break;
            }
            System.out.println("setControlFramePeriod(ControlFrame.Control_3_General) completed with "+retVal.toString());
            // Timer.delay(.001);
        } while (retries-- > 0);

        // General Status - Applied Motor Output, fault Information, Limit Switch Information
        // can be larger (longer period) for Followers
        // default is 10 msec

        setOneStatusFramePeriod(StatusFrameEnhanced.Status_1_General, Math.min(10,mActiveFramePeriod), 100);

        // start in a known state
        mLastRunWasActive = true;
        setFramePeriods(mActiveFramePeriod);
        // switchToActive();
    }

    boolean setOneStatusFramePeriod(StatusFrameEnhanced statusFrame, int framePeriod, int longesttimeout){
        int timeout = 1;
        ErrorCode retVal;

        do {
            retVal = mFXMotor.setStatusFramePeriod(statusFrame, framePeriod, 1);
            timeout++;
        } while(!retVal.equals(ErrorCode.OK) && timeout <= longesttimeout);

        // if (timeout>1)
        {
            System.out.println("leaving setOneStatusFramePeriod "+statusFrame.toString()+" after setting "+timeout+" times with final status "+retVal.toString());
        }
        return retVal.equals(ErrorCode.OK);
    }

    public void switchToActive(){

        if (!mLastRunWasActive){
            // setFramePeriods(mActiveFramePeriod);
        }
        mLastRunWasActive = true;
    }

    public void switchToDormant(){
        if (mLastRunWasActive){
            // setFramePeriods(mDormantFramePeriod);
        }
        mLastRunWasActive = false;
    }

    private void setFramePeriods(int framePeriod){
        double start = Timer.getFPGATimestamp();
        // complete list

        // General Control frame for motor control
        // must be kept low (20 msec?) for proper motor function
        // set above
        // mFXMotor.setControlFramePeriod(ControlFrame.Control_3_General, Math.min(20,mActiveFramePeriod));

        // Advanced Control frame for motor control
        // no documentation found, will keep low
        // set above
        // mFXMotor.setControlFramePeriod(ControlFrame.Control_4_Advanced, Math.min(20,mActiveFramePeriod));
        
        // Control frame for adding trajectory points from Stream object
        // no documentation found
        // feature not used
        // mFXMotor.setControlFramePeriod(ControlFrame.Control_6_MotProfAddTrajPoint, framePeriod);
        
        // General Status - Applied Motor Output, fault Information, Limit Switch Information
        // can be larger (longer period) for Followers
        // default is 10 msec
        // set above
        // mFXMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 10, kTimeout);

        // Feedback for selected sensor on primary PID[0]
        // Selected Sensor Position (PID 0), Selected Sensor Velocity (PID 0), 
        // Brushed Supply Current Measurement, Sticky Fault Information
        // can be larger (longer period) for Followers
        // default is 20
        setOneStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 18, 100);
        // mFXMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 18/*framePeriod*/, kTimeout);
        
        //Quadrature sensor
        // default > 100 msec
        // not used
        // mFXMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, framePeriod, kTimeout);

        // Analog sensor, motor controller temperature, and voltage at input leads
        // default > 100 msec
        // temp might be of interest otherwise not used
        // mFXMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, framePeriod, kTimeout);

        // Miscellaneous signals
        // no documentation
        setOneStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, framePeriod, 100);
        // mFXMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_6_Misc, framePeriod, kTimeout);

        // Communication status
        // no documentation
        setOneStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, framePeriod, 100);
        // mFXMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_7_CommStatus, kTimeout);

        // Pulse width sensor
        // default > 100 msec
        // not used
        // mFXMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, framePeriod, kTimeout);

        // Motion profile buffer status
        // not used
        // mFXMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_9_MotProfBuffer, framePeriod, kTimeout);

        // Brushless Current Status Includes Stator and Supply Current for Talon FX
        // default 50 msec
        setOneStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 18, 100);
        // mFXMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, 18 /*framePeriod*/, kTimeout);
        
        // Motion Profiling/Motion Magic Information
        // default > 100 msec
        setOneStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, framePeriod, 100);
        // mFXMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_Targets, framePeriod, kTimeout);

        // Gadgeteer status
        // mFXMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, framePeriod, kTimeout);
        
        // Selected Sensor Position (Aux PID 1)
        // Selected Sensor Velocity (Aux PID 1)
        // default > 100 msec
        // not used
        // mFXMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, framePeriod, kTimeout);

        // Primary PID
        setOneStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, framePeriod, 100);
        // mFXMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, framePeriod, kTimeout);

        // Auxiliary PID
        // not used
        // mFXMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, framePeriod, kTimeout);

        // Firmware & API
        // mFXMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_15_FirmwareApiStatus,framePeriod,kTimeout);

        // MotionProfile Targets for Auxiliary PID1
        // not used
        // mFXMotor.setStatusFramePeriod(StatusFrame.Status_17_Targets1,framePeriod,kTimeout);

        System.out.println("setFramePeriods took "+(Timer.getFPGATimestamp()-start));
    }
}