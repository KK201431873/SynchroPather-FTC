package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.Synchronizer;
import org.firstinspires.ftc.teamcode.synchropather.systems.__util__.TimeSpan;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftPlan;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.LiftState;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.movements.ConstantVelocityLift;
import org.firstinspires.ftc.teamcode.synchropather.systems.lift.movements.ContinuousVelocityLift;

public class LiftTestAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Motor lift = new Motor(hardwareMap, "lift", Motor.GoBILDA.RPM_312);


        // Lift movements
        ConstantVelocityLift lift1 = new ConstantVelocityLift(new TimeSpan(0, 3),
                new LiftState(0),
                new LiftState(2000)
        );

        ContinuousVelocityLift lift2 = new ContinuousVelocityLift(new TimeSpan(3, 7),
                new LiftState(2000),
                new LiftState(8000)
        );

        ContinuousVelocityLift lift3 = new ContinuousVelocityLift(new TimeSpan(7, 10),
                new LiftState(8000),
                new LiftState(0)
        );


        // Lift plan
        LiftPlan liftPlan = new LiftPlan(lift,
                lift1,
                lift2,
                lift3
        );


        // Pass into the Synchronizer
        Synchronizer synchronizer = new Synchronizer(
                liftPlan
        );

        waitForStart();


        // Main update loop
        synchronizer.start();
        while (opModeIsActive() && synchronizer.update()) {
            // TODO: Whatever you need to do (e.g. send telemetry, update subsystems, etc.)
        }
        synchronizer.stop();

    }

}
