/**
 * Copyright (c) 2025 Automaducks - FTC 23206
 * All rights reserved.
 */
package org.firstinspires.ftc.teamcode;

import com.automaducks.pond.commands.*;
import com.automaducks.pond.subsystems.IDrivetrain;
import com.automaducks.pond.common.Pose2D;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.integration.*;
import org.firstinspires.ftc.teamcode.subsystems.*;

/** @noinspection unused*/
@TeleOp(name = "Drive Operation")
public class OperationDrive extends org.firstinspires.ftc.teamcode.integration.OperationBase {

    /**
     * Add your robot initialization here.
     * <p>
     * This is a good place to initialize the robot subsystems. Just make sure no robot movement is allowed.
     */
    @Override
    protected void onInit() {

        org.firstinspires.ftc.teamcode.integration.HardwareMapAccessor hardwareMapAccessor = new org.firstinspires.ftc.teamcode.integration.HardwareMapAccessor(hardwareMap);

        robotSubsystems.add(new org.firstinspires.ftc.teamcode.subsystems.DeadWheelsLocalizer(hardwareMapAccessor, logger));
        robotSubsystems.add(new org.firstinspires.ftc.teamcode.subsystems.ArmController(hardwareMapAccessor, logger));
        robotSubsystems.add(new org.firstinspires.ftc.teamcode.subsystems.MecanumDrive(hardwareMapAccessor, logger));
    }

    /**
     * Add any code that just needs to run once just prior the operation starting.
     * <p>
     * This is a good place to setup any driving commands.
     */
    @Override
    protected void onStart(){

        // Setup driving commands.
        IDrivetrain drivetrain = robotSubsystems.findFirst(IDrivetrain.class);
        org.firstinspires.ftc.teamcode.subsystems.ArmController armController = robotSubsystems.findFirst(org.firstinspires.ftc.teamcode.subsystems.ArmController.class);

        // Chassis movement
        Command chassisMovement =
            Commands.dynamic()
                .when(CommandConditionBuilder::Always)
                .execute(() -> drivetrain.setPower(new Pose2D(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x)))
                .build("JoystickCommand");

        commandScheduler.runPeriodically(chassisMovement);

        // Arm Movement
        Command armMovement =
            Commands.anyOf("ArmControl",
                Commands.dynamic()
                    .when(c -> c.buttonWasJustPressed(() -> gamepad1.a))
                    .execute(() -> armController.moveToAngle(Math.toRadians(90.0), 0.8))
                    .build("RaiseArm"),
                Commands.dynamic()
                    .when(c -> c.buttonWasJustPressed(() -> gamepad1.b))
                    .execute(() -> armController.moveToAngle(Math.toRadians(0), 0.4))
                    .build("LowerArm"));

        commandScheduler.runPeriodically(armMovement);
    }

    /**
     * Any additional periodic code which is not already being handled by subsystems and commands.
     * <p>
     * IMPORTANT: do not execute anything that blocks the execution in this method!
     */
    @Override
    protected void onPeriodic() {

    }

    /**
     * Run any code before having the robot to stop here.
     */
    @Override
    protected void onStop() {

    }
}