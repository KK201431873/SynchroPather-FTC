package org.firstinspires.ftc.teamcode.synchropather.systems.lift;

import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.synchropather.systems.MovementType;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Movement;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.superclasses.Plan;

/**
 * Object containing a sequence of Movements for the lift.
 */
public class LiftPlan extends Plan<LiftState> {

    private final Motor lift;

    /**
     * Creates a new LiftPlan object with the given Movements.
     */
    public LiftPlan(Motor lift, Movement... movements) {
        super(MovementType.LIFT, movements);
        this.lift = lift;
    }


    /**
     * Controls the rotation output of the robot to the LiftState at the elapsedTime.
     */
    @Override
    public void loop() {
        // Get the setpoint (using built-in function) and the process value from the motor encoder
        double setpoint = getCurrentState().getPosition();
        double processValue = lift.getCurrentPosition();

        // e(t) = sp - pv
        double error = setpoint - processValue;

        // Proportional control law u(t) = kP*e(t)
        double kP = 0.01;
        double output = kP*error;

        lift.set(output);

    }

    /**
     * Halts the lift subsystem.
     */
    @Override
    public void stop() {
        lift.stopMotor();
    }

}
