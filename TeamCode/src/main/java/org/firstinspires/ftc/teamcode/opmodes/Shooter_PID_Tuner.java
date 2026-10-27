package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;
import java.util.Locale;

/**
 * A TeleOp for live-tuning the Intake's PIDF coefficients.
 *
 * --- CONTROLS (Gamepad 1 - Driver) ---
 * Left Stick: Drive/Strafe | Right Stick: Turn
 * Circle Button:   Intake FORWARD
 * Square Button:   Intake REVERSE
 * Triangle Button: Intake OFF
 *
 * --- CONTROLS (Gamepad 2 - Tuner) ---
 * D-Pad Up/Down:    Tune P
 * D-Pad Right/Left: Tune I
 *
 *
 *
 * Circle / Cross:   Tune D
 * L / R Bumpers:    Tune F
 * Triangle / Square: Adjust Forward RPM
 * SHARE Button:     Toggle UDP Debug
 */
@Utility(name = "Shooter PID Tuner")
public class Shooter_PID_Tuner extends OpMode {

    private RobotHardware robot;
    private Shooter shooter;
    private Drivetrain drivetrain;


    private MovingStatistics loopTimes = new MovingStatistics(100);
    private ElapsedTime loopTimer = new ElapsedTime();

    // Tuning increments
    private static final double P_INCREMENT = 0.001;
    private static final double I_INCREMENT = 0.0001;
    private static final double D_INCREMENT = 0.0001;
    private static final double F_INCREMENT = 0.001;
    private static final double SHOOTER_RPM_INCREMENT = 200.0;

    private Intake.FeederPosition FeederPosition = Intake.FeederPosition.DOWN;
    private Intake.FeederPosition PreviousFeederPosition = Intake.FeederPosition.DOWN;

    private double INTAKE_SPEED_INCREMENT = 0.05;

    private double current_intake_speed = 0.0;

    private boolean isIntakeMode = true;


    @Override
    public void init() {
        robot = new RobotHardware();
        robot.init(hardwareMap);
        shooter = new Shooter(robot);
        drivetrain = new Drivetrain(robot);


        // Enable debug by default
        shooter.setDebug(true);

        telemetry.addLine("Shooter PID Tuner Initialized.");
        telemetry.addLine("G1: Drive & Intake | G2: Tune PIDF");
        telemetry.update();
    }

    @Override
    public void loop() {
        loopTimer.reset();

        for (LynxModule module : robot.allHubs) {
            module.clearBulkCache();
        }

        // G1: Driving & Intake Toggles (Matching TeleOp Main)
        drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x * 1.1, -gamepad1.right_stick_x);


        if (gamepad1.dpadUpWasPressed()) {
            shooter.flyWheel1.incrementP(P_INCREMENT);
        }
        if (gamepad1.dpadDownWasPressed()) {
            shooter.flyWheel1.incrementP(-P_INCREMENT);
        }
        if (gamepad1.dpadRightWasPressed()) {
            shooter.flyWheel1.incrementI(I_INCREMENT);
        }
        if (gamepad1.dpadLeftWasPressed()) {
            shooter.flyWheel1.incrementI(-I_INCREMENT);
        }
        if (gamepad1.bWasPressed()) {
            shooter.flyWheel1.incrementD(D_INCREMENT);
        }
        if (gamepad1.aWasPressed()) {
            shooter.flyWheel1.incrementD(-D_INCREMENT);
        }
        if (gamepad1.yWasPressed()) {
            shooter.flyWheel1.incrementF(F_INCREMENT);
        }
        if (gamepad1.xWasPressed()) {
            shooter.flyWheel1.incrementF(-F_INCREMENT);
        }
        if (gamepad2.dpadUpWasPressed()) {
            shooter.flyWheel2.incrementP(P_INCREMENT);
        }
        if (gamepad2.dpadDownWasPressed()) {
            shooter.flyWheel2.incrementP(-P_INCREMENT);
        }
        if (gamepad2.dpadRightWasPressed()) {
            shooter.flyWheel2.incrementI(I_INCREMENT);
        }
        if (gamepad2.dpadLeftWasPressed()) {
            shooter.flyWheel2.incrementI(-I_INCREMENT);
        }
        if (gamepad2.bWasPressed()) {
            shooter.flyWheel2.incrementD(D_INCREMENT);
        }
        if (gamepad2.aWasPressed()) {
            shooter.flyWheel2.incrementD(-D_INCREMENT);
        }
        if (gamepad2.yWasPressed()) {
            shooter.flyWheel2.incrementF(F_INCREMENT);
        }
        if (gamepad2.xWasPressed()) {
            shooter.flyWheel2.incrementF(-F_INCREMENT);
        }
        if (gamepad1.rightBumperWasPressed()) {
            shooter.flyWheel1.incrementRPM(SHOOTER_RPM_INCREMENT);
        }
        if (gamepad1.leftBumperWasPressed()) {
            shooter.flyWheel1.incrementRPM(-SHOOTER_RPM_INCREMENT);
        }
        if (gamepad2.rightBumperWasPressed()) {
            shooter.flyWheel2.incrementRPM(SHOOTER_RPM_INCREMENT);
        }
        if (gamepad2.leftBumperWasPressed()) {
            shooter.flyWheel2.incrementRPM(-SHOOTER_RPM_INCREMENT);
        }

                // Subsystem Heartbeats
        drivetrain.update();
        shooter.update();

        // Telemetry Update
        updateTelemetry();

        loopTimes.add(loopTimer.nanoseconds());
    }

    private void updateTelemetry() {


        telemetry.addData("Loop (Hz)", "%.1f", 1 / (loopTimes.getMean() / 1e9));
        telemetry.addData("Debug (SHARE)", shooter.isDebugEnabled() ? "ON" : "OFF");
        telemetry.addLine(String.format(Locale.US, "Flywheel 1 P:%.5f I:%.5f D:%.5f F:%.5f",
                shooter.flyWheel1.getkP(), shooter.flyWheel1.getkI(), shooter.flyWheel1.getkD(), shooter.flyWheel1.getkF()));
        telemetry.addLine(String.format(Locale.US, "Flywheel 2 P:%.5f I:%.5f D:%.5f F:%.5f",
                shooter.flyWheel2.getkP(), shooter.flyWheel2.getkI(), shooter.flyWheel2.getkD(), shooter.flyWheel2.getkF()));
        telemetry.addData("Flywheel 1 RPM", "Tgt:%.0f | Act:%.0f", shooter.flyWheel1.getTargetRPM(), shooter.flyWheel1.getCurrentRPM());
        telemetry.addData("Flywheel 2 RPM", "Tgt:%.0f | Act:%.0f", shooter.flyWheel2.getTargetRPM(), shooter.flyWheel2.getCurrentRPM());

        telemetry.addLine("\n--- PS4 CONTROLS ---");
        telemetry.addLine("G1: Circle:FWD | Square:REV | Triangle:OFF");
        telemetry.addLine("G2: DPAD U/D:P | DPAD R/L:I | Circle/Cross:D | Triangle/Square:F");
        telemetry.addLine("G2: SHARE:Toggle Debug | Bumpers:Adjust RPM Target");


        telemetry.update();
    }

    @Override
    public void stop() {
        drivetrain.stop();
        shooter.stop();
    }
}
