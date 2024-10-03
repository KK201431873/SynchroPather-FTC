package org.firstinspires.ftc.teamcode.synchropather.systems.lift.movements;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.calculators.StretchedDisplacementCalculator;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.RobotState;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftConstants;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftState;

/**
 * A Lift Movement that linearly interpolates with a continuous velocity function.
 */
public class ContinuousVelocityLift extends Movement {

    /**
     * Takes care of the smoothed velocity calculations.
     */
    private StretchedDisplacementCalculator calculator;

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
     * Create a new ContinuousVelocityLift Movement with the given start and end states that runs for the given TimeSpan.
     * @param timeSpan
     * @param start
     * @param end
     */
    public ContinuousVelocityLift(TimeSpan timeSpan, LiftState start, LiftState end) {
        super(timeSpan, MovementType.LIFT);
        this.start = start;
        this.end = end;

        // Init calculator
        calculator = new StretchedDisplacementCalculator(
                end.getPosition()-start.getPosition(),
                timeSpan,
                LiftConstants.MAX_VELOCITY,
                LiftConstants.MAX_ACCELERATION
        );
        minDuration = calculator.getMinDuration();
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
        double displacement = calculator.getDisplacement(elapsedTime);
        return new LiftState(start.getPosition() + displacement);
    }

    @Override
    /**
     * @param elapsedTime The current elapsed time.
     * @return The desired velocity LiftState at the given elapsed time.
     */
    public RobotState getVelocity(double elapsedTime) {
        return new LiftState(calculator.getVelocity(elapsedTime));
    }

    @Override
    /**
     * @param elapsedTime The current elapsed time.
     * @return The desired acceleration LiftState at the given elapsed time.
     */
    public RobotState getAcceleration(double elapsedTime) {
        return new LiftState(calculator.getAcceleration(elapsedTime));
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
        return "SmoothedVelocityLift";
    }
}
