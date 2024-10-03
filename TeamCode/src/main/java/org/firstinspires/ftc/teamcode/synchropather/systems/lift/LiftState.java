package org.firstinspires.ftc.teamcode.synchropather.systems.lift;

import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.RobotState;

public class LiftState extends RobotState {

    /**
     * The position of this LiftState, in ticks.
     */
    private final double position;

    /**
     * Creates a new LiftState with the given position.
     * @param position the given position in ticks.
     */
    public LiftState(double position) {
        this.position = position;
    }

    /**
     * @return the position of this LiftState.
     */
    public double getPosition() {
        return position;
    }

    @Override
    /**
     * @return a String containing the position in ticks.
     */
    public String toString() {
        return String.format("%s ticks", position);
    }

    @Override
    /**
     * @return "Lift"
     */
    public String getDisplayName() {
        return "Lift";
    }
}
