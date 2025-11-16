/**
 * Copyright (c) 2025 Automaducks - FTC 23206
 * All rights reserved.
 */

package org.firstinspires.ftc.teamcode.subsystems;

import com.automaducks.pond.common.Pose2D;
import com.automaducks.pond.common.PoseAndVelocity;
import com.automaducks.pond.kinematics.DeadWheelsKinematics;
import com.automaducks.pond.subsystems.*;
import com.automaducks.pond.utility.*;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.integration.HardwareMapAccessor;

/**
 * Tracks robot position through dead wheels.
 */
public class DeadWheelsLocalizer extends SubsystemBase implements ILocalizer {

    private final DeadWheelsKinematics deadWheelsKinematics;

    private final DcMotor deadWheelRight;
    private final DcMotor deadWheelLeft;
    private final DcMotor deadWheelCenter;

    /**
     * Constructs a new instance of the SubsystemBase class.
     *
     * @param logger telemetry logger.
     * @param hardwareMap robot hardware map.
     */
    public DeadWheelsLocalizer(HardwareMapAccessor hardwareMap, ILogger logger) {
        super(logger);

        this.deadWheelsKinematics = new DeadWheelsKinematics(logger, Configuration.Chassis.DeadWheels);

        this.deadWheelLeft = hardwareMap.getMotor(Configuration.Chassis.MotorRearLeft);
        this.deadWheelRight = hardwareMap.getMotor(Configuration.Chassis.MotorRearRight);
        this.deadWheelCenter = hardwareMap.getMotor(Configuration.Chassis.MotorFrontRight);

        this.deadWheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.deadWheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.deadWheelCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public PoseAndVelocity getLocalization() {
        return this.deadWheelsKinematics.getPositionVelocity();
    }

    @Override
    public void setLocalization(Pose2D pose) {
        this.deadWheelsKinematics.updateAndReset(
            deadWheelLeft.getCurrentPosition(),
            deadWheelRight.getCurrentPosition(),
            deadWheelCenter.getCurrentPosition(),
            pose);
    }

    @Override
    public boolean reacquire() {
        return false;
    }

    @Override
    public void periodic() {
        this.deadWheelsKinematics.update(
            -deadWheelLeft.getCurrentPosition(),
            -deadWheelRight.getCurrentPosition(),
            deadWheelCenter.getCurrentPosition());

        this.logger.displayData("Localization", getLocalization().pose);
    }
}
