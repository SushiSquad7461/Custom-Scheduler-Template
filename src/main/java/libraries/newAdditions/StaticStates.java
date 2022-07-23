package libraries.newAdditions;

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

    public static enum CollecterState {
        ASSESSING,
        BACKING,
        COLLECTING,
        DISABLING,
        HOLDING,
        MANUAL_CONTROLLING,
        FEEDING
    }
}
