package frc.robot.subsystems;

import libraries.cheesylib.loops.Loop.Phase;
import libraries.cheesylib.subsystems.Subsystem;
import libraries.newAdditions.StaticStates.ExampleState;

public class ExampleSubsytem extends Subsystem<ExampleState> {
    private static ExampleSubsytem sInstance = null;

    public static ExampleSubsytem getInstance(String caller) {
        if (sInstance == null) {
            sInstance = new ExampleSubsytem(caller);
        } else {
            sInstance.printUsage(caller);
        }
        return sInstance;
    }

    private ExampleSubsytem(String caller) {
        printUsage(caller);
    }

    @Override
    public void start(Phase phase) {        
    }

    @Override
    public void onLoop() {        
    }

    @Override
    public void stop() {        
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
