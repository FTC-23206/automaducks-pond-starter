/**
 * Copyright (c) 2025 Automaducks - FTC 23206
 * All rights reserved.
 */
package org.firstinspires.ftc.samplecode.integration;

import com.automaducks.pond.commands.CommandScheduler;
import com.automaducks.pond.subsystems.*;
import com.automaducks.pond.utility.*;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

/**
 * Base class for all robot operation modes.
 * <p>
 * It initializes all robot subsystems and define basic operation stages:
 * - onInit : executes on OpMode initialization
 * - onRun : executes on OpMode run
 * - onStop : executes on OpMode stop
 *  * <p>
 *  * IMPORTANT: This file is not expected to be changed by teams.
 */
public abstract class OperationBase extends LinearOpMode {

    private final String logTag;
    private final ElapsedTime runtime = new ElapsedTime();
    private final AverageTracker periodicTimeTracker = new AverageTracker(100);
    private final List<LynxModule> lynxModules = new ArrayList<>();

    protected final ITelemetryLogger logger = new TelemetryLogger(telemetry);
    protected final SubsystemHub robotSubsystems = new SubsystemHub(logger);
    protected final CommandScheduler commandScheduler = new CommandScheduler(logger);

    /**
     * Common class initialization.
     */
    protected OperationBase() {
        this.logTag = LogUtil.getLogTag("op", this);
    }

    /**
     * Runs the robot operation mode by splitting it in 3 sections:
     * -onStart
     * -onRun
     * -onStop
     * @throws InterruptedException when interrupted.
     */
    @Override
    public void runOpMode() throws InterruptedException {

        try {
            // Init
            initInternal();

            while (!isStarted()) {
                sleep(20);
            }

            // Run
            startInternal();

            while (!isStopRequested()) {
                periodicInternal();
            }
        }
        finally {

            // Stop
            stopInternal();
        }
    }

    /**
     * Initializes basic robot functions.
     * @throws InterruptedException when interrupted.
     */
    protected abstract void onInit() throws InterruptedException;

    /**
     * Last chance execution before periodic run.
     * @throws InterruptedException when interrupted.
     */
    protected abstract void onStart() throws InterruptedException;

    /**
     * Runs the operation mode.
     * @throws InterruptedException when interrupted.
     */
    protected abstract void onPeriodic() throws InterruptedException;

    /**
     * Executes code prior stopping the operation.
     */
    protected abstract void onStop() throws InterruptedException;


    /**
     * Robot initialization.
     */
    private void initInternal() throws InterruptedException {

        // Log init started
        this.logger.displayData("Status", "Initializing...");

        // Reset run timer
        this.runtime.reset();

        // Allow custom initialization for child classes
        this.onInit();

        // Initialize Lynx Modules Last
        this.lynxModules.addAll(hardwareMap.getAll(LynxModule.class));

        // Log end
        this.logger.displayData("Status", "Initialized!");
        this.logger.flush(true);
    }

    /**
     * Completes initialization.
     * <p>
     * OPTIONAL: Initialize common robot subsystems that may cause movement here.
     */
    private void startInternal() throws InterruptedException {

        // Reset run timer
        this.runtime.reset();

        // Child classes before run.
        this.onStart();

        // Enable caching for hardware reads, this speeds up encoder reads which
        // is useful, especially in autonomous modes. See link below for more information:
        // https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html
        //
        // IMPORTANT: Keep this call after component initialization, since blocking methods such as
        // findZeroPositions require updated reads from hardware.
        for (LynxModule lynxModule : this.lynxModules) {
            lynxModule.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    /**
     * Periodic execution.
     */
    private void periodicInternal() throws InterruptedException {

        // Clear the hardware state cache prior the periodic tasks, this ensures the periodic
        // tasks will reads the most current values from hardware
        for (LynxModule lynxModule : this.lynxModules) {
            lynxModule.clearBulkCache();
        }

        // Runs all robot real time systems
        ElapsedTime periodicElapsed = new ElapsedTime();

        this.robotSubsystems.periodic();
        this.commandScheduler.periodic();

        // Calls child operations periodic
        this.onPeriodic();

        double averageElapsed = periodicTimeTracker.add(periodicElapsed.milliseconds());

        // Log status and flush telemetry
        this.logger.displayData(
            "Status",
            "Running for %s (p=%.1f ms)",
            runtime,
            averageElapsed);

        this.logger.flush(false);

        // Allow other threads to process
        idle();
    }

    /**
     * Before stopping.
     */
    private void stopInternal() {

        // Stop
        try {
            this.onStop();
        }
        catch (Exception e){
            logger.logException(getLogTag(), e, "Error on cleanup.");
            logger.displayData("Status", "Stopped");
            logger.flush(true);
        }
    }

    /**
     * Gets the standard log tag for this class.
     * @return standard log tag.
     */
    protected String getLogTag() {
        return logTag;
    }
}