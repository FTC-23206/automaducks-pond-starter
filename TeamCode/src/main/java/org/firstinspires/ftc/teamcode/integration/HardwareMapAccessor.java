/**
 * Copyright (c) 2025 Automaducks - FTC 23206
 * All rights reserved.
 */
package org.firstinspires.ftc.samplecode.integration;

import com.automaducks.pond.utility.HardwareMapAccessorBase;
import com.automaducks.pond.kinematics.MotorParams;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareMapAccessor extends HardwareMapAccessorBase<DcMotor> {

    private final HardwareMap hardwareMap;

    /**
     * Hardware map helper.
     *
     * @param hardwareMap FTC Robot hardware map.
     */
    public HardwareMapAccessor(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    /**
     * Gets and configures a motor from the hardware map.
     *
     * @param motorParams motor parameters.
     * @return the motor reference.
     */
    @Override
    public DcMotor getAndConfigureMotor(MotorParams motorParams) {

        DcMotor motor = hardwareMap.get(DcMotor.class, motorParams.name);

        switch (motorParams.zeroPowerBehavior) {
            case BRAKE:
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                break;
            case COAST:
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                break;
            default:
                throw new RuntimeException("Invalid option:" + motorParams.zeroPowerBehavior);
        }

        switch (motorParams.mode) {
            case RUN_TO_POSITION:
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motor.setTargetPosition(0);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
            case RUN_USING_ENCODER:
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                break;
            case RUN_WITHOUT_ENCODER:
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                break;
        }

        if (motorParams.invertMotor) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else
        {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        return motor;
    }

    public DcMotor getMotor(MotorParams motorParams){
        return hardwareMap.get(DcMotor.class, motorParams.name);
    }
}