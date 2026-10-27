package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Utility;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.BeamBreak;

/**
 * Iterative TeleOp OpMode to test and display the 5 Beam Break sensors:
 * - Right Side: Beam 1, Beam 2
 * - Middle:     Beam 3
 * - Left Side:  Beam 5, Beam 6
 *
 * --- CONTROLS ---
 * Gamepad 1 'A' Button: Toggle Plot Client Debug Mode
 */
@Utility(name = "Beam Break Test")
public class TeleOp_BeamBreak_Test extends OpMode {

    private RobotHardware robot;
    private BeamBreak beamBreak;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing Beam Break Test...");

        robot = new RobotHardware();
        robot.init(hardwareMap);
        beamBreak = new BeamBreak(robot);

        // Enable UDP debug plotting by default
        beamBreak.setDebug(true);

        telemetry.addLine("Beam Break Test Initialized.");
        telemetry.addLine("Press 'A' on Gamepad 1 to toggle UDP Plot Client.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Clear bulk read caches for expansion/control hubs
        if (robot.allHubs != null) {
            for (LynxModule module : robot.allHubs) {
                module.clearBulkCache();
            }
        }

        // Toggle debug mode on gamepad 1 'A' press
        if (gamepad1.aWasPressed()) {
            beamBreak.setDebug(!beamBreak.isDebugEnabled());
        }

        // Update beam break subsystem (sends plot data if debug enabled)
        beamBreak.update();

        // Update Driver Station Telemetry
        updateTelemetry();
    }

    private void updateTelemetry() {
        telemetry.addData("Status", "Running");
        telemetry.addData("UDP Debug Plotter", beamBreak.isDebugEnabled() ? "ENABLED" : "DISABLED");
        telemetry.addLine("--- Beam Receiver States ---");
        telemetry.addData("Beam 1 (Right 1)", beamBreak.isSensorOneOpen() ? "OPEN" : "CLOSED");
        telemetry.addData("Beam 2 (Right 2)", beamBreak.isSensorTwoOpen() ? "OPEN" : "CLOSED");
        telemetry.addData("Beam 3 (Middle)",  beamBreak.isSensorThreeOpen() ? "OPEN" : "CLOSED");
        telemetry.addData("Beam 5 (Left 1)",  beamBreak.isSensorFiveOpen() ? "OPEN" : "CLOSED");
        telemetry.addData("Beam 6 (Left 2)",  beamBreak.isSensorSixOpen() ? "OPEN" : "CLOSED");
        telemetry.addLine("--- Detection ---");
        telemetry.addData("Ball Present", beamBreak.isBallPresent() ? "YES" : "NO");
        telemetry.addData("Ball Type", beamBreak.getBallType());
        telemetry.update();
    }

    @Override
    public void stop() {
        beamBreak.close();
        telemetry.addData("Status", "Stopped");
        telemetry.update();
    }
}
