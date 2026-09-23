package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * This is a simple TeleOp program to test a standard servo and two continuous rotation (CR) servos.
 * - Standard servo should be configured with the name "servo".
 * - First CR servo should be configured with the name "cr_servo1".
 * - Second CR servo should be configured with the name "cr_servo2".
 *
 * --- Controls ---
 * - Gamepad 1 Right Stick (Up/Down): Controls the position of the standard servo.
 * - Gamepad 1 D-Pad (Up/Down): Incrementally controls the power of both CR servos simultaneously (in opposite directions).
 * - Gamepad 1 'Y' Button: Resets CR servo power to 0.
 *
 * Telemetry is displayed to show the current target position and power.
 */
@TeleOp(name = "Servo Test", group = "Test")
//@Disabled
public class ServoTestOpMode extends OpMode {

    // --- Hardware ---
    private Servo testServo;
    private CRServo testCrServo1;
    private CRServo testCrServo2;

    // --- Control Variables ---
    private double servoPosition = 0.5; // Start at the center position
    private double crServoPower = 0.0;
    private final double SERVO_INCREMENT = 0.005; // Rate of change for standard servo
    private final double CR_SERVO_INCREMENT = 0.1; // Rate of change for CR servo

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // --- Map the standard servo ---
        try {
            testServo = hardwareMap.get(Servo.class, "servo");
            telemetry.addData("Status", "Standard Servo 'servo' found.");
        } catch (Exception e) {
            telemetry.addData("Status", "ERROR: Could not find servo named 'servo'.");
            testServo = null;
        }

        // --- Map the first CR servo ---
        try {
            testCrServo1 = hardwareMap.get(CRServo.class, "right_booster_servo");
            telemetry.addData("Status", "CR Servo 'right_booster_servo' found.");
        } catch (Exception e) {
            telemetry.addData("Status", "ERROR: Could not find CR servo named 'right_booster_servo'.");
            testCrServo1 = null;
        }

        // --- Map the second CR servo ---
        try {
            testCrServo2 = hardwareMap.get(CRServo.class, "left_booster_servo");
            telemetry.addData("Status", "CR Servo 'left_booster_servo' found.");
        } catch (Exception e) {
            telemetry.addData("Status", "ERROR: Could not find CR servo named 'left_booster_servo'.");
            testCrServo2 = null;
        }

        telemetry.addLine("\n--- Ready to Test ---");
        telemetry.addLine("Right Stick (U/D): Control Standard Servo Position");
        telemetry.addLine("D-Pad (U/D): Increment/Decrement CR Servos Power");
        telemetry.addLine("Y Button: Reset CR Servos Power to 0");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        // Nothing to do here for this simple test
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
        // Set the servos to their initial positions/powers when the OpMode starts
        if (testServo != null) {
            testServo.setPosition(servoPosition);
        }
        if (testCrServo1 != null) {
            testCrServo1.setPower(crServoPower);
        }
        if (testCrServo2 != null) {
            testCrServo2.setPower(crServoPower);
        }
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        // --- Standard Servo Control (Right Stick) ---
        if (testServo != null) {
            // Get the value from the right stick's Y-axis. It's inverted, so -1 is up.
            double rightStickValue = -gamepad1.right_stick_y;
            // Adjust the servo position based on the stick value
            servoPosition += rightStickValue * SERVO_INCREMENT;
            // Clip the position to the valid servo range [0.0, 1.0]
            servoPosition = Range.clip(servoPosition, 0.0, 1.0);
            // Set the servo's target position
            testServo.setPosition(servoPosition);
        }

        // --- CR Servo Control (D-pad) ---
        if (gamepad1.dpadUpWasPressed()) {
            crServoPower += CR_SERVO_INCREMENT;
        } else if (gamepad1.dpadDownWasPressed()) {
            crServoPower -= CR_SERVO_INCREMENT;
        }

        // Add a button to reset the power to zero for convenience
        if (gamepad1.y) {
            crServoPower = 0.0;
        }

        // Clip the power to the valid range [-1.0, 1.0]
        crServoPower = Range.clip(crServoPower, -1.0, 1.0);

        if (testCrServo1 != null) {
            // Set the first CR servo's power directly
            testCrServo1.setPower(crServoPower);
        }
        if (testCrServo2 != null) {
            // Set the second CR servo's power to the opposite value
            testCrServo2.setPower(crServoPower);
        }

        // --- Telemetry ---
        telemetry.addLine("--- Standard Servo ('servo') ---");
        if (testServo != null) {
            telemetry.addData("Target Position", "%.3f", servoPosition);
        } else {
            telemetry.addData("Status", "Not Found");
        }

        telemetry.addLine("\n--- CR Servos ---");
        if (testCrServo1 != null) {
            telemetry.addData("CR Servo 1 ('right_roller_bed_servo') Power", "%.3f", crServoPower);
        } else {
            telemetry.addData("CR Servo 1 Status", "Not Found");
        }

        if (testCrServo2 != null) {
            telemetry.addData("CR Servo 2 ('left_roller_bed_servo') Power", "%.3f", crServoPower);
        } else {
            telemetry.addData("CR Servo 2 Status", "Not Found");
        }

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        // Ensure motors are stopped
        if (testCrServo1 != null) {
            testCrServo1.setPower(0);
        }
        if (testCrServo2 != null) {
            testCrServo2.setPower(0);
        }
        telemetry.addData("Status", "OpMode Stopped.");
        telemetry.update();
    }
}
