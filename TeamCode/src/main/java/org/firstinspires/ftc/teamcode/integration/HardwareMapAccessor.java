/**
 * Copyright (c) 2025 Automaducks - FTC 23206
 * All rights reserved.
 */
package org.firstinspires.ftc.teamcode.integration;

import com.automaducks.pond.control.PidParamsSimple;
import com.automaducks.pond.utility.HardwareMapAccessorBase;
import com.automaducks.pond.kinematics.MotorParams;
import com.automaducks.pond.utility.ILogger;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

public class HardwareMapAccessor extends HardwareMapAccessorBase<DcMotor> {

    private final HardwareMap hardwareMap;
    private final ILogger logger;

    /**
     * Hardware map helper.
     *
     * @param hardwareMap FTC Robot hardware map.
     */
    public HardwareMapAccessor(HardwareMap hardwareMap, ILogger logger) {
        this.hardwareMap = hardwareMap;
        this.logger = logger;
    }

    /**
     * Gets and configures a motor from the hardware map.
     *
     * @param motorParams motor parameters.
     * @return the motor reference.
     */
    @Override
    public DcMotor getAndConfigureMotor(MotorParams motorParams) {

        DcMotorEx motorEx = hardwareMap.get(DcMotorEx.class, motorParams.name);

        switch (motorParams.zeroPowerBehavior) {
            case BRAKE:
                motorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                break;
            case COAST:
                motorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                break;
            default:
                throw new RuntimeException("Invalid option:" + motorParams.zeroPowerBehavior);
        }

        switch (motorParams.mode) {
            case RUN_TO_POSITION:
                motorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorEx.setTargetPosition(0);
                motorEx.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case RUN_USING_ENCODER:
                motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            case RUN_WITHOUT_ENCODER:
                motorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
        }

        if (motorParams.invert) {
            motorEx.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else
        {
            motorEx.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        // PID
        PidParamsSimple positionPid = motorParams.positionPid;
        if (!positionPid.isZero()) {
            motorEx.setPositionPIDFCoefficients(positionPid.kp);
        }

        PidParamsSimple velocityPid = motorParams.velocityPid;
        if (!velocityPid.isZero()) {
            motorEx.setVelocityPIDFCoefficients(
                velocityPid.kp,
                velocityPid.ki,
                velocityPid.kd,
                velocityPid.kf);
        }

        if (motorParams.disable) {
            motorEx.setMotorDisable();
        }

        PIDFCoefficients effectivePositionPid = motorEx.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        PIDFCoefficients effectiveVelocityPid = motorEx.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);

        // Return
        this.logger.logDebug("hma", "Returned motor '%s': mode=%s, invert=%s, PIDPos=(%.2f), PIDVel=(%.2f, %.2f, %.2f, %.2f)",
            motorParams.name,
            motorParams.mode,
            motorParams.invert,
            effectivePositionPid.p,
            effectiveVelocityPid.p,
            effectiveVelocityPid.i,
            effectiveVelocityPid.d,
            effectiveVelocityPid.f);

        return motorEx;
    }

    /**
     * Retrieve a motor by name.
     */
    public DcMotor getMotor(MotorParams motorParams){
        return hardwareMap.get(DcMotor.class, motorParams.name);
    }

    /**
     * Retrieve a servo by name.
     * @noinspection unused
     */
    public Servo getServo(String name) {
        return hardwareMap.get(Servo.class, name);
    }

    /**
     * Retrieve an LED by name.
     * @noinspection unused
     */
    public LED getLED(String name) { return hardwareMap.get(LED.class, name);}

    /**
     * Retrieve a Limelight3A by name.
     * @noinspection unused
     */
    public Limelight3A getLimelight3A(String name) { return hardwareMap.get(Limelight3A.class, name);}

    /**
     * Retrieve a Sensor by name.
     * @noinspection unused
     */
    public TouchSensor getSensor(String name) { return hardwareMap.get(TouchSensor.class, name);}

    /**
     * Retrieve a VisionPortal by camera name and processors (or null if not found).
     * @noinspection unused
     */
    public VisionPortal getVisionPortal(String cameraName, VisionProcessor... processors) {

        WebcamName webcamName = hardwareMap.tryGet(WebcamName.class, cameraName);

        return webcamName != null && webcamName.isAttached()
            ? VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, cameraName), processors)
            : null;
    }
}