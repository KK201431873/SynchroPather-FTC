package org.firstinspires.ftc.teamcode.synchropather.systems.lift.movements;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.RobotState;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftState;

/**
 * A Lift Movement that linearly interpolates with a constant velocity.
 */
public class ConstantVelocityLift extends Movement {

    private final double minDuration;

    /**
     * The initial lift position.
     */
    private final LiftState start;

    /**
     * The final lift position.
     */
    private final LiftState end;

    /**
     * Create a new ConstantVelocityLift Movement with the given start and end states that runs for the given TimeSpan.
     * @param timeSpan
     * @param start
     * @param end
     */
    public ConstantVelocityLift(TimeSpan timeSpan, LiftState start, LiftState end) {
        super(timeSpan, MovementType.LIFT);
        this.start = start;
        this.end = end;
        this.minDuration = Math.abs(start.getPosition()-end.getPosition()) / LiftConstants.MAX_VELOCITY;
        if (timeSpan.getDuration() < minDuration) throw new RuntimeException("TimeSpan duration is less than minDuration!");
    }

    @Override
    public double getMinDuration() {
        return minDuration;
    }

    @Override
    /**
     * @param elapsedTime The current elapsed time.
     * @return The desired LiftState at the given elapsed time.
     */
    public RobotState getState(double elapsedTime) {
        // clamp within timeSpan
        elapsedTime = timeSpan.clamp(elapsedTime);

        // scale to a value between 0 and 1
        double t = (elapsedTime - timeSpan.getStartTime()) / timeSpan.getDuration();

        // use linear interpolation formula
        double position = start.getPosition() + t * (end.getPosition() - start.getPosition());

        return new LiftState(position);
    }

    @Override
    /**
     * @param elapsedTime The current elapsed time.
     * @return The desired velocity LiftState at the given elapsed time.
     */
    public RobotState getVelocity(double elapsedTime) {
        // use velocity formula
        double velocity = (end.getPosition() - start.getPosition()) / timeSpan.getDuration();

        return new LiftState(velocity);
    }

    @Override
    /**
     * @param elapsedTime The current elapsed time.
     * @return The desired acceleration LiftState at the given elapsed time.
     */
    public RobotState getAcceleration(double elapsedTime) {
        // acceleration is zero
        return new LiftState(0);
    }

    @Override
    /**
     * @return the initial LiftState of this Movement.
     */
    public RobotState getStartState() {
        return start;
    }

    @Override
    /**
     * @return the final LiftState of this Movement.
     */
    public RobotState getEndState() {
        return end;
    }

    @Override
    public String getDisplayName() {
        return "ConstantVelocityLift";
    }
}
