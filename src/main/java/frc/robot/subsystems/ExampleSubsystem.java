package frc.robot.subsystems;

import SushiFrcLib.Scheduler.Loops.Loop.Phase;
import SushiFrcLib.Scheduler.Subsystems.Subsystem;
import frc.robot.constants.StaticStates.ExampleState;

public class ExampleSubsystem extends Subsystem<ExampleState> {
    //Subsystem Creation
    private static ExampleSubsystem sInstance = null;

    public static ExampleSubsystem getInstance(String caller) {
        if (sInstance == null) {
            sInstance = new ExampleSubsystem(caller);
        } else {
            sInstance.printUsage(caller);
        }
        return sInstance;
    }

    private ExampleSubsystem(String caller) {
        printUsage(caller);
    }

    @Override
    public void start(Phase phase) {    
        synchronized (ExampleSubsystem.this) {
        }
    }

    @Override
    public void periodic() { }

    @Override
    public void stop() { }

    @Override
    public void outputTelemetry() { }
}
