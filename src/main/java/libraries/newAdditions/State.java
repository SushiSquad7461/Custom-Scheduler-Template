package libraries.newAdditions;

// IMPORTANT
// i did a litle bit of trolling and realized mid way through writing this class
// that their is a better way to do it
// however i dont want to remove this in case i might need it latter
// so it is heir for the lols
// HOWEVER THIS CLASS IS CURRENTLY USLESS
public class State<SubsystemState extends Enum<SubsystemState>> {
    // BROKEN CLASS
    private SubsystemState currentState;
    private SubsystemState wantedState;

    public State() {
    }

        // BROKEN CLASS


    public synchronized SubsystemState getCurrentState() {
        return currentState;
    }
    

        // BROKEN CLASS


    public SubsystemState transferState() {
        currentState = wantedState;
        return currentState;
    }

        // BROKEN CLASS
    // BROKEN CLASS
    // BROKEN CLASS


    public void setWantedState(SubsystemState newState) {
        wantedState = newState;
    }
}
