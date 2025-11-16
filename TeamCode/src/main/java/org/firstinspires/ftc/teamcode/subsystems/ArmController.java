/**
 * Copyright (c) 2025 Automaducks - FTC 23206
 * All rights reserved.
 */
package org.firstinspires.ftc.teamcode.subsystems;

import com.automaducks.pond.kinematics.AngularActuatorCalculator;
import com.automaducks.pond.subsystems.SubsystemBase;
import com.automaducks.pond.utility.ILogger;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Configuration;
import org.firstinspires.ftc.teamcode.integration.HardwareMapAccessor;

/**
 * Sample robot arm controller.
 */
public class ArmController extends SubsystemBase {

    private final AngularActuatorCalculator angularActuatorCalculator;
    private final DcMotor armMotor;

    /**
     * Creates a new instance of the arm controller.
     * @param hardwareMap hardware maps accessor.
     * @param logger logger instance.
     */
    public ArmController(HardwareMapAccessor hardwareMap, ILogger logger) {

        super(logger);

        // Init Hardware
        this.angularActuatorCalculator = new AngularActuatorCalculator(Configuration.Arm.PivotActuation);
        this.armMotor = hardwareMap.getAndConfigureMotor(Configuration.Arm.MotorArm);
    }

    /**
     * Moves the arm to a defined angle.
     * @param power movement power.
     * @param angle desired angle.
     */
    public void moveToAngle(double angle, double power) {

        int position = angularActuatorCalculator.anglesToPulses(angle);

        armMotor.setTargetPosition(position);
        armMotor.setPower(power);
    }
}
