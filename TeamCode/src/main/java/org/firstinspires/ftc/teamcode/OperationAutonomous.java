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

/** @noinspection unused*/
@Autonomous(name = "Auto Operation")
public class OperationAutonomous extends org.firstinspires.ftc.teamcode.integration.OperationBase {

    @Override
    protected void onInit() {

        org.firstinspires.ftc.teamcode.integration.HardwareMapAccessor hardwareMapAccessor = new org.firstinspires.ftc.teamcode.integration.HardwareMapAccessor(hardwareMap);

        robotSubsystems.add(new org.firstinspires.ftc.teamcode.subsystems.DeadWheelsLocalizer(hardwareMapAccessor, logger));
        robotSubsystems.add(new org.firstinspires.ftc.teamcode.subsystems.ArmController(hardwareMapAccessor, logger));
        robotSubsystems.add(new org.firstinspires.ftc.teamcode.subsystems.MecanumDrive(hardwareMapAccessor, logger));

        // Auto Only
        robotSubsystems.add(new HolonomicController(org.firstinspires.ftc.teamcode.Configuration.Autonomous.HolonomicConfig, logger));
    }

    @Override
    protected void onStart() {

        Pose2D initialPose = new Pose2D(0, 0, 0);
        FollowTrajectoryCommandParams trajectoryParams = org.firstinspires.ftc.teamcode.Configuration.Autonomous.TrajectoryParams;

        ITimedTrajectory trajectory = org.firstinspires.ftc.teamcode.roadrunner.RoadRunner.createTrajectory(initialPose, b -> b
            .lineToX(600)
            .build());

        Command command = Commands.followTrajectory("Test", trajectory, trajectoryParams, robotSubsystems);

        commandScheduler.runOnce(command);

    }

    @Override
    protected void onPeriodic() {

    }

    @Override
    protected void onStop() {

    }
}