// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;

import SushiFrcLib.ChesyLibUtil.CrashTracker;
import SushiFrcLib.Constants.SushiConstants;
import SushiFrcLib.DependencyInjection.RobotName;
import SushiFrcLib.Scheduler.Loops.Looper;
import SushiFrcLib.Scheduler.Subsystems.SubsystemManager;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    // Name 
    private String mClassName;

    // Subsystems
    private SubsystemManager mSubsystemManager;
    private JSticks mJSticks;
    private Shooter mShooter;

    private Looper mSubsystemLooper = new Looper(SushiConstants.SCHEDULER.LOOPPERIOD, Thread.NORM_PRIORITY + 1);

    @Override
    public void robotInit() {
        System.out.println("robotInit() begins");
        Timer.delay(.05); // give rest of system 50 msec to come up first
        mClassName = this.getClass().getSimpleName();
        
        // Get name and set constants
        RobotName.getInstance();
        Constants.setup();

        // Initializing subsystems
        mShooter = Shooter.getInstance(mClassName);
        mJSticks = JSticks.getInstance(mClassName);

        // Create subsystem manager and add all subsystems it will manage
        mSubsystemManager = SubsystemManager.getInstance(mClassName);
        mSubsystemManager.initializeSubsystemManager(
            (int) (SushiConstants.SCHEDULER.LOOPPERIODMS),
            Arrays.asList(
                // List of subsystems
                mShooter,
                mJSticks
            )
        );

        // ask each subsystem to register itself
        mSubsystemManager.registerEnabledLoops(mSubsystemLooper);

        System.out.println("RobotInit() ends");
    }

    @Override
    public void robotPeriodic() {
        // Ensure timely updates when graphing values etc.
        NetworkTableInstance.getDefault().flush();
    }

    @Override
    public void autonomousInit() {
        System.out.println("*********** AutonomousInit() begins ***********");
        System.out.println("AutonomousInit() ends");
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        System.out.println("*********** TeleopInit() begins ***********");

        try {
            mSubsystemLooper.stop();
            mSubsystemLooper.start();
        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }

        System.out.println("TeleopInit() ends");
    }

    @Override
    public void teleopPeriodic() { }

    @Override
    public void disabledInit() {
        System.out.println("*********** disabledInit() begins ***********");
        try {
            System.gc();
            mSubsystemLooper.stop();
            mSubsystemLooper.start();

        } catch (Throwable t) {
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
        System.out.println("disabledInit() ends");
    }

    @Override
    public void disabledPeriodic() { }

    @Override
    public void testInit() { }

    @Override
    public void testPeriodic() { }
}
