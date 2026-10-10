package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.oldcode.Lift;
import org.firstinspires.ftc.teamcode.util.IntakeRoller;

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
 * Circle / Cross:   Tune D
 * L / R Bumpers:    Tune F
 * Triangle / Square: Adjust Forward RPM
 * SHARE Button:     Toggle UDP Debug
 */
@TeleOp(name = "Intake PID Tuner", group = "Test")
public class Intake_PID_Tuner extends OpMode {

    private RobotHardware robot;
    private Intake intake;
    private Drivetrain drivetrain;

    private IntakeRoller roller;
    private Lift lift;

    private MovingStatistics loopTimes = new MovingStatistics(100);
    private ElapsedTime loopTimer = new ElapsedTime();

    // Tuning increments
    private static final double P_INCREMENT = 0.001;
    private static final double I_INCREMENT = 0.0001;
    private static final double D_INCREMENT = 0.0001;
    private static final double F_INCREMENT = 0.001;

    private Intake.FeederPosition FeederPosition = Intake.FeederPosition.DOWN;
    private Intake.FeederPosition PreviousFeederPosition = Intake.FeederPosition.DOWN;

    private double INTAKE_SPEED_INCREMENT = 0.05;

    private double current_intake_speed = 0.0;

    private boolean isIntakeMode = true;

    @Override
    public void init() {
        robot = new RobotHardware();
        robot.init(hardwareMap);
        intake = new Intake(robot);
        drivetrain = new Drivetrain(robot);
        roller = new IntakeRoller(robot.intakeRoller1, robot.intakeRoller2);

        // Enable debug by default
        intake.setDebug(true);
        
        telemetry.addLine("Intake PID Tuner Initialized.");
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

        
        if (gamepad1.b) intake.changeState(Intake.IntakeState.FORWARD);
        if (gamepad1.x) intake.changeState(Intake.IntakeState.REVERSE);
        if (gamepad1.y) intake.changeState(Intake.IntakeState.OFF);

        // G2: PIDF Tuning
        IntakeRoller roller = intake.getRollerController();

        if (gamepad1.rightBumperWasPressed()) {
            if (isIntakeMode) {
                isIntakeMode = false;
            }
            else {
                isIntakeMode = true;
            }
        }
        
        // Debug Toggle (Back Button)
        if (gamepad2.backWasPressed()) {
            intake.setDebug(!roller.isDebugEnabled());
        }

        // RPM Adjustment (Y: +100, X: -100)
        if (gamepad2.yWasPressed()) {
            intake.setForwardRPM(intake.getForwardRPM() + 100);
        }

        if (gamepad2.xWasPressed()) {
            intake.setForwardRPM(intake.getForwardRPM() - 100);
        }

        // P Tuning
        if (gamepad2.dpad_up)    roller.adjustP(P_INCREMENT);
        if (gamepad2.dpad_down)  roller.adjustP(-P_INCREMENT);
        
        // I Tuning
        if (gamepad2.dpad_right) roller.adjustI(I_INCREMENT);
        if (gamepad2.dpad_left)  roller.adjustI(-I_INCREMENT);
        
        // D Tuning
        if (gamepad2.b)          roller.adjustD(D_INCREMENT);
        if (gamepad2.a)          roller.adjustD(-D_INCREMENT);
        
        // F Tuning
        if (gamepad2.right_bumper) roller.adjustF(F_INCREMENT);
        if (gamepad2.left_bumper)  roller.adjustF(-F_INCREMENT);

        if (!isIntakeMode) {
            if (gamepad1.dpadUpWasPressed()) {
                FeederPosition = Intake.FeederPosition.UP;
            }

            if (gamepad1.dpadDownWasPressed()) {
                FeederPosition = Intake.FeederPosition.DOWN;
            }
        } else {
            if (gamepad1.dpadUpWasPressed()) {
                current_intake_speed += INTAKE_SPEED_INCREMENT;
                roller.setPower(current_intake_speed);
            }
            if (gamepad1.dpadDownWasPressed()) {
                current_intake_speed -= INTAKE_SPEED_INCREMENT;
                roller.setPower(current_intake_speed);
            }
            if (gamepad1.dpadRightWasPressed()) {
                roller.setPower(0);
            }
        }

        if (PreviousFeederPosition != FeederPosition) {
            intake.setFeederPosition(FeederPosition);
        }

        PreviousFeederPosition = FeederPosition;

        // Subsystem Heartbeats
        drivetrain.update();
        intake.update();

        // Telemetry Update
        updateTelemetry();

        loopTimes.add(loopTimer.nanoseconds());
    }

    private void updateTelemetry() {
        IntakeRoller roller = intake.getRollerController();

        telemetry.addData("Loop (Hz)", "%.1f", 1 / (loopTimes.getMean() / 1e9));
        telemetry.addData("Debug (SHARE)", roller.isDebugEnabled() ? "ON" : "OFF");
        telemetry.addLine(String.format(Locale.US, "P:%.5f I:%.5f D:%.5f F:%.5f", 
                roller.getkP(), roller.getkI(), roller.getkD(), roller.getkF()));
        telemetry.addData("Forward RPM (Triangle/Square)", "%.0f", intake.getForwardRPM());
        telemetry.addData("RPM", "Tgt:%.0f | Act:%.0f", roller.getTargetRPM(), roller.getCurrentRPM());
        telemetry.addData("State", intake.getIntakeState().toString());
        telemetry.addData("Feeder Position", intake.getFeederPosition());
        telemetry.addData("Feeder Position Ticks", intake.getFeederPositionTicks());
        telemetry.addData("Tuner Mode", isIntakeMode ? "Intake Mode" : "Feeder Mode");
        telemetry.addData("Current Intake Power", current_intake_speed);

        telemetry.addLine("\n--- PS4 CONTROLS ---");
        telemetry.addLine("G1: Circle:FWD | Square:REV | Triangle:OFF");
        telemetry.addLine("G2: DPAD U/D:P | DPAD R/L:I | Circle/Cross:D | Bumpers:F");
        telemetry.addLine("G2: SHARE:Toggle Debug | Triangle/Square:Adjust Forward RPM");

        
        telemetry.update();
    }

    @Override
    public void stop() {
        drivetrain.stop();
        intake.stop();
    }
}
