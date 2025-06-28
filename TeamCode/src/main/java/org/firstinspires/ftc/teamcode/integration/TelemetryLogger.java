/**
 * Copyright (c) 2025 Automaducks - FTC 23206
 * All rights reserved.
 */
package org.firstinspires.ftc.teamcode.integration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.automaducks.pond.utility.TelemetryLoggerBase;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

/**
 * Multi target logger: logs to DriverHub, LogCat and FTC Dashboard Graphs.
 * <p>
 * IMPORTANT: This file is not expected to be changed by teams.
 */
public class TelemetryLogger extends TelemetryLoggerBase {

    private final Telemetry telemetry;
    private final FtcDashboard ftcDashboard = FtcDashboard.getInstance();
    private TelemetryPacket ftcDashboardPacket = new TelemetryPacket();

    /**
     * Constructs a new telemetry logger.
     * @param telemetry the FTC SDK Telemetry reference.
     */
    public TelemetryLogger(Telemetry telemetry) {

        this.telemetry = telemetry;
        this.populateGraphKeys(new ArrayList<>());
    }

    @Override
    protected void displayDataInternal(String key, Object value) {
        telemetry.addData(key, value);
    }

    @Override
    protected void graphDataInternal(String key, Object value) {
        this.ftcDashboardPacket.put(key, value);
    }

    @Override
    protected void logDebugInternal(String tag, String format, Object... args) {
        RobotLog.dd(tag, format, args);
    }

    @Override
    protected void logInformationInternal(String tag, String format, Object... args) { RobotLog.ii(tag, format, args); }

    @Override
    protected void logExceptionInternal(String tag, Exception e, String format, Object... args) { RobotLog.ee(tag, e, format, args); }

    @Override
    protected void flushInternal() {
        // Send telemetry values
        this.ftcDashboard.sendTelemetryPacket(ftcDashboardPacket);
        this.telemetry.update();

        ftcDashboardPacket = new TelemetryPacket();
    }
}