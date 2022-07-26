package frc.robot.constants;

public class StaticStates {
    public static enum SolenoidState {
        EXTEND(true),
        RETRACT(false);

        private final boolean state;

        private SolenoidState(boolean state) {
            this.state = state;
        }

        public boolean get() {
            return state;
        }
    }

    public static enum ShooterState {
        TARMAC,
        FENDER,
        DISABLED
    }

    public static enum JStickState {
        READING,
        DISABLED
    }

    public static enum RobotState {
        HOLD,
        DISABLED
    }
}
