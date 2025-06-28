/**
 * Copyright (c) 2025 Automaducks - FTC 23206
 * All rights reserved.
 */
package org.firstinspires.ftc.teamcode.subsystems;

import com.automaducks.pond.common.Pose2D;
import com.automaducks.pond.kinematics.MecanumInverseKinematics;
import com.automaducks.pond.kinematics.MecanumWheelKinematicsData;
import com.automaducks.pond.subsystems.*;
import com.automaducks.pond.utility.*;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.integration.HardwareMapAccessor;

/**
 * Mecanum chassis subsystem implementation.
 */
public class MecanumDrive extends SubsystemBase implements IDrivetrain {

    private final DcMotor frontLeft;
    private final DcMotor rearLeft;
    private final DcMotor frontRight;
    private final DcMotor rearRight;

    private final Pose2D targetPower = new Pose2D();

    public MecanumDrive(HardwareMapAccessor hardwareMap, ITelemetryLogger logger) {

        super(logger);

        this.frontLeft = hardwareMap.getAndConfigureMotor(Configuration.Chassis.MotorFrontLeft);
        this.frontRight = hardwareMap.getAndConfigureMotor(Configuration.Chassis.MotorFrontRight);
        this.rearLeft = hardwareMap.getAndConfigureMotor(Configuration.Chassis.MotorRearLeft);
        this.rearRight = hardwareMap.getAndConfigureMotor(Configuration.Chassis.MotorRearRight);
    }

    @Override
    public void setPower(Pose2D power) {

        this.logger.logDebug(getLogTag(), "TargetPower=%s", power);

        this.targetPower.copyFrom(power);
    }

    @Override
    public void periodic() {

        MecanumWheelKinematicsData<Double> wheelsData = MecanumInverseKinematics.calculate(this.targetPower);

        this.frontLeft.setPower(wheelsData.frontLeft);
        this.frontRight.setPower(wheelsData.frontRight);
        this.rearLeft.setPower(wheelsData.rearLeft);
        this.rearRight.setPower(wheelsData.rearRight);

        logger.logDebug(getLogTag(),"Mecanum Power: %s", wheelsData);
    }
}
