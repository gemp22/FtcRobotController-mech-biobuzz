package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.UdpClientPlot;

/**
 * Subsystem for managing 5 Beam Break sensors:
 * - Right Side: Beam 1, Beam 2
 * - Middle:     Beam 3
 * - Left Side:  Beam 5, Beam 6
 */
public class BeamBreak {

    public enum BallType {
        POLLEN,
        NECTAR,
        NONE
    }

    private final DigitalChannel beamSensorOne;   // Right 1
    private final DigitalChannel beamSensorTwo;   // Right 2
    private final DigitalChannel beamSensorThree; // Middle
    private final DigitalChannel beamSensorFive;  // Left 1
    private final DigitalChannel beamSensorSix;   // Left 2

    // --- Debug & Plotting ---
    private boolean isDebugEnabled = false;
    private UdpClientPlot plotClient;
    private String plotClientHost = "192.168.43.100";
    private int plotClientPort = 7778;

    public BeamBreak(RobotHardware robot) {
        this.beamSensorOne = robot.beamSensorOne;
        this.beamSensorTwo = robot.beamSensorTwo;
        this.beamSensorThree = robot.beamSensorThree;
        this.beamSensorFive = robot.beamSensorFive;
        this.beamSensorSix = robot.beamSensorSix;

        if (beamSensorOne != null) beamSensorOne.setMode(DigitalChannel.Mode.INPUT);
        if (beamSensorTwo != null) beamSensorTwo.setMode(DigitalChannel.Mode.INPUT);
        if (beamSensorThree != null) beamSensorThree.setMode(DigitalChannel.Mode.INPUT);
        if (beamSensorFive != null) beamSensorFive.setMode(DigitalChannel.Mode.INPUT);
        if (beamSensorSix != null) beamSensorSix.setMode(DigitalChannel.Mode.INPUT);
    }

    /**
     * Enables or disables the UDP plot client for debugging.
     */
    public void setDebug(boolean enable) {
        if (enable && !this.isDebugEnabled) {
            this.isDebugEnabled = true;
            if (plotClient == null) {
                plotClient = new UdpClientPlot(plotClientHost, plotClientPort);
                if (plotClient.isInitialized()) {
                    long initTime = System.currentTimeMillis();
                    plotClient.sendYLimits(initTime, 8.0, -0.5);
                    plotClient.sendYUnits(initTime + 1, "State (1=Open, 0=Closed)");

                    plotClient.sendSeriesNameLine(initTime + 2, "Right Beam 1", 1);
                    plotClient.sendSeriesNameLine(initTime + 3, "Right Beam 2", 2);
                    plotClient.sendSeriesNameLine(initTime + 4, "Middle Beam 3", 3);
                    plotClient.sendSeriesNameLine(initTime + 5, "Left Beam 5", 4);
                    plotClient.sendSeriesNameLine(initTime + 6, "Left Beam 6", 5);

                    plotClient.sendTextMarker(initTime + 2, "BeamBreak 5-Sensor Debug Enabled", "top");
                }
            }
        } else if (!enable && this.isDebugEnabled) {
            this.isDebugEnabled = false;
            close();
        }
    }

    public void setDebug(boolean enable, String host, int port) {
        this.plotClientHost = host;
        this.plotClientPort = port;
        setDebug(enable);
    }

    public boolean isDebugEnabled() {
        return isDebugEnabled;
    }

    // --- Sensor State Getters ---
    // DigitalChannel.getState() returns true for HIGH (open/unbroken) and false for LOW (closed/broken).
    // Returns true if open (or null), false if closed (beam broken).

    public boolean isSensorOneOpen() {
        return beamSensorOne == null || beamSensorOne.getState();
    }

    public boolean isSensorTwoOpen() {
        return beamSensorTwo == null || beamSensorTwo.getState();
    }

    public boolean isSensorThreeOpen() {
        return beamSensorThree == null || beamSensorThree.getState();
    }

    public boolean isSensorFiveOpen() {
        return beamSensorFive == null || beamSensorFive.getState();
    }

    public boolean isSensorSixOpen() {
        return beamSensorSix == null || beamSensorSix.getState();
    }

    public boolean isSensorOneClosed() { return !isSensorOneOpen(); }
    public boolean isSensorTwoClosed() { return !isSensorTwoOpen(); }
    public boolean isSensorThreeClosed() { return !isSensorThreeOpen(); }
    public boolean isSensorFiveClosed() { return !isSensorFiveOpen(); }
    public boolean isSensorSixClosed() { return !isSensorSixOpen(); }

    /**
     * Returns true if specified sensor (1, 2, 3, 5, 6) is open.
     */
    public boolean isSensorOpen(int index) {
        switch (index) {
            case 1: return isSensorOneOpen();
            case 2: return isSensorTwoOpen();
            case 3: return isSensorThreeOpen();
            case 5: return isSensorFiveOpen();
            case 6: return isSensorSixOpen();
            default: return true;
        }
    }

    public boolean isSensorClosed(int index) {
        return !isSensorOpen(index);
    }

    /**
     * Returns true if ANY active beam sensor is broken (closed by a ball).
     */
    public boolean isBallPresent() {
        int[] activeIndices = {1, 2, 3, 5, 6};
        for (int i : activeIndices) {
            if (isSensorClosed(i)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Classifies the ball based on sensor coverage:
     * - NECTAR: Ball spans across both Right side (1 or 2) and Left side (5 or 6).
     * - POLLEN: Ball is present but does not span both sides.
     * - NONE: No ball detected.
     */
    public BallType getBallType() {
        if (isBallPresent()) {
            boolean rightTriggered = isSensorOneClosed() || isSensorTwoClosed();
            boolean leftTriggered = isSensorFiveClosed() || isSensorSixClosed();

            if (rightTriggered && leftTriggered) {
                return BallType.NECTAR;
            }
            return BallType.POLLEN;
        }
        return BallType.NONE;
    }

    /**
     * Heartbeat method to update debug plot outputs.
     */
    public void update() {
        if (isDebugEnabled && plotClient != null) {
            long time = System.currentTimeMillis();

            double val1 = isSensorOneOpen() ? 1.0 : 0.0;
            double val2 = isSensorTwoOpen() ? 2.0 : 1.1;
            double val3 = isSensorThreeOpen() ? 3.0 : 2.1;
            double val5 = isSensorFiveOpen() ? 4.0 : 3.1;
            double val6 = isSensorSixOpen() ? 5.0 : 4.1;

            plotClient.sendLineY(time, val1, 1);
            plotClient.sendLineY(time, val2, 2);
            plotClient.sendLineY(time, val3, 3);
            plotClient.sendLineY(time, val5, 4);
            plotClient.sendLineY(time, val6, 5);

            plotClient.sendKeyValue(time, "Beam 1 (Right 1)", isSensorOneOpen() ? "OPEN" : "CLOSED");
            plotClient.sendKeyValue(time, "Beam 2 (Right 2)", isSensorTwoOpen() ? "OPEN" : "CLOSED");
            plotClient.sendKeyValue(time, "Beam 3 (Middle)", isSensorThreeOpen() ? "OPEN" : "CLOSED");
            plotClient.sendKeyValue(time, "Beam 5 (Left 1)", isSensorFiveOpen() ? "OPEN" : "CLOSED");
            plotClient.sendKeyValue(time, "Beam 6 (Left 2)", isSensorSixOpen() ? "OPEN" : "CLOSED");
            plotClient.sendKeyValue(time, "Is Ball Present?", isBallPresent() ? "YES" : "NO");
            plotClient.sendKeyValue(time, "Ball Type", getBallType().toString());
        }
    }

    /**
     * Closes the UDP plot client socket.
     */
    public void close() {
        if (plotClient != null) {
            plotClient.close();
            plotClient = null;
        }
    }
}
