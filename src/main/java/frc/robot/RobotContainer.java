package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.constants.Constants.OI;
import frc.robot.constants.StaticStates.RobotState;
import frc.robot.subsystems.Shooter;
import libraries.cheesylib.loops.Loop.Phase;
import libraries.cheesylib.subsystems.Subsystem;
import frc.robot.constants.StaticStates.ShooterState;

public class RobotContainer extends Subsystem<RobotState>{
    //Subsystem Creation
    private static RobotContainer sInstance = null;

    private XboxController driver;
    private XboxController operator;
    private Shooter shooter;

    public static RobotContainer getInstance(String caller) {
        if (sInstance == null) {
            sInstance = new RobotContainer(caller);
        } else {
            sInstance.printUsage(caller);
        }
        return sInstance;
    }

    private RobotContainer(String caller) {
        shooter = Shooter.getInstance("RobotContainer");
        driver = new XboxController(OI.DRIVER);
        operator = new XboxController(OI.OPERATOR);

        new JoystickButton(driver, XboxController.Button.kA.value)
            .whenPressed(() -> shooter.setWantedState(ShooterState.FENDER, "Robot Container"));

        new JoystickButton(driver, XboxController.Button.kX.value)
            .whenPressed(() -> shooter.setWantedState(ShooterState.TARMAC, "Robot Container"));

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
