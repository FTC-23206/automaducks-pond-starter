/**
 * Copyright (c) 2025 Automaducks - FTC 23206
 * All rights reserved.
 */
package org.firstinspires.ftc.teamcode;

import com.automaducks.pond.commands.*;
import com.automaducks.pond.common.*;
import com.automaducks.pond.kinematics.*;
import com.automaducks.pond.subsystems.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.integration.*;
import org.firstinspires.ftc.teamcode.roadrunner.*;
import org.firstinspires.ftc.teamcode.subsystems.*;

/** @noinspection unused*/
@Autonomous(name = "Auto Operation")
public class OperationAutonomous extends OperationBase {

    @Override
    protected void onInit() {

        HardwareMapAccessor hardwareMapAccessor = new HardwareMapAccessor(hardwareMap, logger);

        robotSubsystems.add(new DeadWheelsLocalizer(hardwareMapAccessor, logger));
        robotSubsystems.add(new ArmController(hardwareMapAccessor, logger));
        robotSubsystems.add(new MecanumDrive(hardwareMapAccessor, logger));

        // Auto Only
        robotSubsystems.add(new HolonomicController(Configuration.Autonomous.HolonomicConfig, logger));
    }

    @Override
    protected void onStart() {

        Pose2D initialPose = new Pose2D(0, 0, 0);
        CommandFactory commandFactory = new CommandFactory(this.robotSubsystems, this.logger);
        FollowTrajectoryCommandParams trajectoryParams = Configuration.Autonomous.TrajectoryParams;

        ITimedTrajectory trajectory = RoadRunner.createTrajectory(initialPose, b -> b
            .lineToX(60)
            .build());

        Command command = commandFactory.followTrajectory("Test", trajectory, trajectoryParams);

        commandScheduler.runOnce(command);

    }

    @Override
    protected void onPeriodic() {

    }

    @Override
    protected void onStop() {

    }
}
