package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.IntakeRoller;

import java.util.Locale;

/**
 * Subsystem Diagnostic & Testing Manager.
 *
 * Select a subsystem test variant using D-Pad UP/DOWN during init_loop().
 * Press START on the Driver Station to run the active test mode.
 */
@Utility(name = "Subsystem Test")
public class Subsystem_Test extends OpMode {

    // --- Subsystems & Hardware ---
    private RobotHardware robot;
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;

    private ElapsedTime loopTimer = new ElapsedTime();
    private MovingStatistics loopTimes = new MovingStatistics(100);

    private static final double INTAKE_POSITION_INCREMENT = 0.05;
    private static final double INTAKE_SPEED_INCREMENT = 0.05;
    private static final double SHOOTER_RPM_INCREMENT = 100.0;


    // --- Selection State ---
    private int selectedTestIndex = 0; // 0=A, 1=B, 2=C, 3=D, 4=E, 5=F, 6=G
    private final String[] variantLabels = {"A", "B", "C", "D", "E", "F", "G"};
    private static final String[] TEST_TITLES = {
            "Drivetrain",
            "Intake",
            "Shooter",
            "Turret",
            "Motors",
            "Servos",
            "Sensors"
    };
    private boolean isDrivetrainEnabled = true;
    private boolean isShooterPIDMode = false;

    @Override
    public void init() {
        telemetry.addData("Status", "Initializing Subsystem Tester...");

        robot = new RobotHardware();
        robot.init(hardwareMap);

        drivetrain = new Drivetrain(robot);
        intake = new Intake(robot);
        // shooter = new Shooter(robot);
        // turret = new Turret(robot);

        drivetrain.resetOdometry();

        selectedTestIndex = 0;

        telemetry.addData("Status", "Ready. Select Subsystem Routine...");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        for (LynxModule module : robot.allHubs) {
            module.clearBulkCache();
        }

        // Handle Subsystem Selection (Vertical DPAD)
        if (gamepad1.dpadDownWasPressed()) {
            selectedTestIndex = (selectedTestIndex + 1) % variantLabels.length;
        }
        if (gamepad1.dpadUpWasPressed()) {
            selectedTestIndex = (selectedTestIndex + variantLabels.length - 1) % variantLabels.length;
        }

        displayMenuUI();
    }

    @Override
    public void start() {
        intake.changeState(Intake.IntakeState.OFF);

        System.out.printf("SUBSYSTEM TEST START: Variant=%s, Title=%s%n",
                variantLabels[selectedTestIndex], TEST_TITLES[selectedTestIndex]);

        telemetry.clearAll();
    }

    @Override
    public void loop() {
        for (LynxModule module : robot.allHubs) {
            module.clearBulkCache();
        }

        loopTimer.reset();

        telemetry.addLine(String.format("=== RUNNING TEST %s: %s ===",
                variantLabels[selectedTestIndex], TEST_TITLES[selectedTestIndex].toUpperCase(Locale.US)));
        telemetry.addLine("------------------------------------------");

        switch (selectedTestIndex) {
            case 0: // Variant A
                runDrivetrainTest();
                break;
            case 1: // Variant B
                runIntakeTest();
                break;
            case 2: // Variant C
                runShooterTest();
                break;
            case 3: // Variant D
                runTurretTest();
                break;
            case 4: // Variant E
                runMotorTest();
                break;
            case 5: // Variant F
                runServoTest();
                break;
            case 6: // Variant G
                runSensorTest();
                break;
        }

        drivetrain.update();
        intake.update();
        shooter.update();

        updateTelemetry();

        loopTimes.add(loopTimer.nanoseconds());
    }



    @Override
    public void stop() {
        intake.stop();
        drivetrain.stop();
        shooter.stop();
    }

    private void displayMenuUI() {
        telemetry.addLine("=== SUBSYSTEM TEST SELECTION ===");
        telemetry.addData("Selected Variant", variantLabels[selectedTestIndex]);
        telemetry.addData("Selected Test", TEST_TITLES[selectedTestIndex]);

        telemetry.addLine("\n=== SUBSYSTEM SELECTION (D-Pad) ===");

        for (int i = 0; i < variantLabels.length; i++) {
            printVariantLine(variantLabels[i], TEST_TITLES[i], i);
        }

        telemetry.update();
    }

    private void printVariantLine(String label, String title, int index) {
        String prefix = (selectedTestIndex == index) ? " > [*] " : "    [  ] ";
        telemetry.addLine(prefix + label + ": " + title);
    }

    private void updateTelemetry() {
        telemetry.addData("Active Subsystem", TEST_TITLES[selectedTestIndex]);
        telemetry.update();
    }

    // =========================================================================
    //  SUBSYSTEM SPECIFIC TEST ROUTINES
    // =========================================================================

    private void runDrivetrainTest() {
        double leftPower = -gamepad1.left_stick_y;
        double rightPower = -gamepad1.right_stick_y;

        drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x * 1.1, -gamepad1.right_stick_x);
        // drivetrain.setPowers(leftPower, rightPower);

        telemetry.addLine("Controls: Left Y -> Left Side | Right Y -> Right Side");
        telemetry.addData("Left Power", String.format(Locale.US, "%.2f", leftPower));
        telemetry.addData("Right Power", String.format(Locale.US, "%.2f", rightPower));
    }

    private void runIntakeTest() {
        if (isDrivetrainEnabled) {
            drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x * 1.1, -gamepad1.right_stick_x);
        }
        if (gamepad1.startWasPressed()) {isDrivetrainEnabled = !isDrivetrainEnabled;}
        if (gamepad1.dpadUpWasPressed()) {
            intake.incrementIntakePosition(INTAKE_POSITION_INCREMENT);
        }
        if (gamepad1.dpadDownWasPressed()) {
            intake.incrementIntakePosition(-INTAKE_POSITION_INCREMENT);
        }
        if (gamepad1.yWasPressed()) {
            intake.setIntakePower(INTAKE_SPEED_INCREMENT);
        }
        if (gamepad1.aWasPressed()) {
            intake.setIntakePower(-INTAKE_SPEED_INCREMENT);
        }


        telemetry.addLine("\n--- INTAKE TEST CONTROLS ---");
        telemetry.addLine("G1: L-Stick:Drive | R-Stick X:Turn");
        telemetry.addLine(String.format(Locale.US, "G1: DPAD U/D:Adjust Position (Inc: %.3f)", INTAKE_POSITION_INCREMENT));
        telemetry.addLine(String.format(Locale.US, "G1: Triangle(Y)/Cross(A):Adjust Power (Inc: %.3f)", INTAKE_SPEED_INCREMENT));
        telemetry.addLine("G1: START:Toggle Drivetrain");
        telemetry.addLine("Is Drivetrain Enabled: " + isDrivetrainEnabled);
        telemetry.addData("Current Position", "%.2f", intake.getIntakePosition());
        telemetry.addData("Current Power", "%.2f", intake.getIntakePower());
        telemetry.addData("Current RPM", intake.getIntakeRPM());
    }
    private void runServoTest() {
    }

    private void runMotorTest() {
    }

    private void runTurretTest() {
    }

    private void runShooterTest() {
        if (isDrivetrainEnabled) {
            drivetrain.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x * 1.1, -gamepad1.right_stick_x);
        }

        if (gamepad1.startWasPressed()) {isDrivetrainEnabled = !isDrivetrainEnabled;}

        if (isShooterPIDMode) {
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
        } else {

        }

    }


    private void runSensorTest() {
        telemetry.addLine("Reading live hardware sensors...");
        // telemetry.addData("IMU Heading", drivetrain.getHeading());
    }

}