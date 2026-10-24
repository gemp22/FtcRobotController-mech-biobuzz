package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.HuskyLensUtil;
import org.firstinspires.ftc.teamcode.util.IntakeRoller;

import java.util.Locale;

/**
 * The Intake subsystem is responsible for collecting balls from the field,
 * transporting them internally, detecting their color, and managing the gates
 * and servos that feed them to the shooter.
 */
public class Intake {

    // =========================================================================
    // ENUMS
    // =========================================================================

    /**
     * Operational state for the intake roller subsystem.
     */
    public enum IntakeState {
        OFF,
        FORWARD,
        REVERSE
    }

    /**
     * Position state for the feeder servos.
     */
    public enum FeederPosition {
        UP,
        DOWN
    }

    /**
     * Height/Deploy position state for the intake assembly.
     */
    public enum IntakePosition {
        UP,
        MID,
        LOW
    }

    // =========================================================================
    // CONSTANTS
    // =========================================================================

    private static final double INTAKE_ROLLER_SPEED = 1.0;
    private static final double INTAKE_ROLLER_SPEED_FORWARD = 1.0;
    private static final double INTAKE_ROLLER_SPEED_BACKWARD = -1.0;

    private static final double CRSERVO_FORWARD_POWER = 1.0;
    private static final double CRSERVO_REVERSE_POWER = -1.0;
    private static final double CRSERVO_STOP_POWER = 0.0;
    private static final double ROLLER_BED_SPEED_INCREMENT = 0.1;
    private static final long GATE_OPEN_DURATION_MS = 3000;

    // Constants for Shake Mode
    private static final long ROLLER_BED_SHAKE_FORWARD_MS = 300;
    private static final long ROLLER_BED_SHAKE_REVERSE_MS = 400;
    private static final double ROLLER_BED_SHAKE_SPEED = 0.7;

    // Gate servo position constants
    private static final double LEFT_GATE_OPEN_POS = 1;
    private static final double LEFT_GATE_CLOSE_POS = 0.279;
    private static final double MID_GATE_OPEN_POS = 0;

    // Intake deployment servo position constants
    private static final double INTAKE_SERVO_UP_POS = 0.1;
    private static final double INTAKE_SERVO_MID_POS = 0.4;
    private static final double INTAKE_SERVO_LOW_POS = 0.55;

    // =========================================================================
    // HARDWARE
    // =========================================================================

    private final IntakeRoller rollerController;
    private final HuskyLensUtil intakeCamera;

    private final Servo feederServo1;
    private final Servo feederServo2;

    private final Servo intakeServo1;
    private final Servo intakeServo2;

    // =========================================================================
    // STATE VARIABLES
    // =========================================================================

    private IntakeState currentIntakeState = IntakeState.FORWARD;
    private IntakeState previousIntakeState = IntakeState.FORWARD;

    private IntakePosition currentIntakePosition = IntakePosition.UP;
    private double previousIntakePosition = 0.0;

    private double previousIntakePower = 0.0;
    private double intakePower = 0.0;

    private FeederPosition currentFeederPosition = FeederPosition.DOWN;
    private double lastFeederUpPosition = 0;

    private double forwardRPM = 2200.0;
    private boolean isCameraSensingEnabled = false;

    // =========================================================================
    // CONSTRUCTOR
    // =========================================================================

    /**
     * Constructs the Intake subsystem and maps hardware components from RobotHardware.
     *
     * @param robot The initialized RobotHardware instance containing hardware mappings.
     */
    public Intake(RobotHardware robot) {
        DcMotorEx motor1 = robot.intakeRoller1;
        DcMotorEx motor2 = robot.intakeRoller2;

        this.feederServo1 = robot.feederServo1;
        this.feederServo2 = robot.feederServo2;

        this.intakeServo1 = robot.intakeServo1;
        this.intakeServo2 = robot.intakeServo2;

        this.intakeCamera = new HuskyLensUtil(robot);
        this.rollerController = new IntakeRoller(motor1, motor2);
    }

    // =========================================================================
    // SUBSYSTEM LIFECYCLE & UPDATE METHODS
    // =========================================================================

    /**
     * Periodic update method called during loop execution. Handles state transitions
     * for the intake roller, calculates power output, and updates hardware heartbeats.
     */
    public void update() {
//        if (currentIntakeState != previousIntakeState) {
//            switch (currentIntakeState) {
//                case FORWARD:
//                    rollerController.setPower(intakePower);
//                    break;
//                case REVERSE:
//                    rollerController.setPower(-intakePower);
//                    break;
//                case OFF:
//                    rollerController.stop();
//                    break;
//                default:
//                    rollerController.stop();
//                    break;
//            }
//            previousIntakeState = currentIntakeState;
//        }
        rollerController.setPower(intakePower);

        rollerController.update();
        intakeCamera.update();
    }

    /**
     * Stops all motors and resets the subsystem state to OFF.
     */
    public void stop() {
        rollerController.stop();
        changeState(IntakeState.OFF);
    }

    // =========================================================================
    // INTAKE ROLLER & STATE CONTROL METHODS
    // =========================================================================

    /**
     * Transitions the intake subsystem to a new operational state (OFF, FORWARD, REVERSE).
     *
     * @param newState The desired IntakeState.
     */
    public void changeState(IntakeState newState) {
        System.out.printf(Locale.US, "INTAKE: Previous state: %s New State: %s %n", currentIntakeState, newState);
        currentIntakeState = newState;
    }

    /**
     * Sets the active power level for the intake roller.
     *
     * @param power The power magnitude to set for the intake roller.
     */
    public void setIntakePower(double power) {
        intakePower = power;
    }

    /**
     * Increments or decrements the current intake power by a specified amount.
     *
     * @param increment The magnitude change to apply to the intake power.
     */
    public void incrementIntakePower(double increment) {
        intakePower += increment;
    }

    /**
     * Returns the current power setting for the intake.
     *
     * @return The active intake power value.
     */
    public double getIntakePower() {
        return intakePower;
    }

    /**
     * Gets the active rotation speed (RPM) from the intake roller velocity controller.
     *
     * @return The current RPM of the roller motors.
     */
    public double getIntakeRPM() {
        return rollerController.getCurrentRPM();
    }

    /**
     * Sets the target RPM for the FORWARD intake state.
     *
     * @param rpm The target RPM.
     */
    public void setForwardRPM(double rpm) {
        this.forwardRPM = rpm;
        rollerController.setRPM(forwardRPM);
    }

    /**
     * Gets the current target RPM for the FORWARD intake state.
     *
     * @return The target RPM.
     */
    public double getForwardRPM() {
        return forwardRPM;
    }

    /**
     * Retrieves the low-level roller PIDF controller utility object.
     *
     * @return The IntakeRoller instance.
     */
    public IntakeRoller getRollerController() {
        return rollerController;
    }

    // =========================================================================
    // SERVO & FEEDER CONTROL METHODS
    // =========================================================================

    /**
     * Sets the physical feeder servos to the specified position (UP or DOWN).
     *
     * @param position The target FeederPosition state.
     */
    public void setFeederPosition(FeederPosition position) {
        if (position == FeederPosition.UP) {
            feederServo1.setPosition(1.0);
            feederServo2.setPosition(1.0);
        } else if (position == FeederPosition.DOWN) {
            feederServo1.setPosition(0.5);
            feederServo2.setPosition(0.5);
        }
    }

    /**
     * Returns the current feeder state as a human-readable string.
     *
     * @return "UP", "DOWN", or "UNKNOWN".
     */
    public String getFeederPosition() {
        if (currentFeederPosition == FeederPosition.UP) {
            return "UP";
        } else if (currentFeederPosition == FeederPosition.DOWN) {
            return "DOWN";
        }
        return "UNKNOWN";
    }

    /**
     * Returns a representation of the feeder position in tick/fraction units.
     *
     * @return Position as a Double value.
     */
    public Double getFeederPositionTicks() {
        if (currentFeederPosition == FeederPosition.UP) {
            return lastFeederUpPosition;
        } else if (currentFeederPosition == FeederPosition.DOWN) {
            return 0.5;
        }
        return 0.1;
    }

    /**
     * Sets the deployment height position of the intake servos (UP, MID, or LOW).
     *
     * @param position The target IntakePosition enum value.
     */
    public void setIntakePosition(IntakePosition position) {
        switch (position) {
            case UP:
                intakeServo1.setPosition(INTAKE_SERVO_UP_POS);
                intakeServo2.setPosition(INTAKE_SERVO_UP_POS);
                break;
            case MID:
                intakeServo1.setPosition(INTAKE_SERVO_MID_POS);
                intakeServo2.setPosition(INTAKE_SERVO_MID_POS);
                break;
            case LOW:
                intakeServo1.setPosition(INTAKE_SERVO_LOW_POS);
                intakeServo2.setPosition(INTAKE_SERVO_LOW_POS);
                break;
        }
    }

    /**
     * Directly updates the internal state tracking for the intake position.
     *
     * @param position The target IntakePosition enum.
     */
    public void setIntakePositions(IntakePosition position) {
        currentIntakePosition = position;
        System.out.printf(Locale.US, "INTAKE: Previous position: %s New position %s %n", currentIntakePosition, position);
    }

    /**
     * Gets the currently tracked intake deployment position enum value.
     *
     * @return Current IntakePosition.
     */
    public IntakePosition getIntakePosition() {
        return currentIntakePosition;
    }

    /**
     * Increments the intake deployment servo position by a relative offset value.
     *
     * @param increment The value to add to the previous intake position offset.
     */
    public void incrementIntakePosition(double increment) {
        intakeServo1.setPosition(previousIntakePosition + increment);
        intakeServo2.setPosition(previousIntakePosition + increment);
    }

    // =========================================================================
    // CAMERA & SENSOR METHODS
    // =========================================================================

    /**
     * Enables or disables state updates and sensing logic from the HuskyLens camera.
     *
     * @param enable True to enable camera sensing, false to disable.
     */
    public void enableCameraSensing(boolean enable) {
        this.isCameraSensingEnabled = enable;
    }

    /**
     * Checks if the intake HuskyLens camera currently detects a ball in view.
     *
     * @return True if a ball is detected.
     */
    public boolean isBallInFront() {
        return intakeCamera.isBallDetected();
    }

    /**
     * Convenience method to get the horizontal pixel X position of the detected target.
     * Prioritizes the center of a cluster if detected, otherwise the best single ball.
     *
     * @return The X-coordinate (0-319), or -1 if nothing is detected.
     */
    public int getFrontBallPositionX() {
        if (intakeCamera.isClusterDetected()) {
            return intakeCamera.getBestCluster().centerX;
        }
        return (intakeCamera.getBestBall() != null) ? intakeCamera.getBestBall().x : -1;
    }

    /**
     * Returns the timestamp (in seconds) of when a ball was last detected by the camera.
     *
     * @return Last detected ball time in seconds.
     */
    public double lastBallTime() {
        return intakeCamera.getLastBallDetected();
    }

    /**
     * Enables or disables debugging for the intake roller PIDF controller.
     *
     * @param enable True to enable debug mode, false to disable.
     */
    public void setDebug(boolean enable) {
        rollerController.setDebug(enable);
    }

    /**
     * Enables debug plotting for a specific color sensor.
     *
     * @param sensorIndex Sensor index to enable debug for (-1 to disable all).
     */
    public void setDebug(int sensorIndex) {
    }

    // =========================================================================
    // TELEMETRY METHODS
    // =========================================================================

    /**
     * Stream intake camera telemetry data to Driver Station telemetry output.
     *
     * @param telemetry Telemetry object from active OpMode.
     */
    public void addIntakeCameraTelemetry(Telemetry telemetry) {
        intakeCamera.addTelemetry(telemetry);
    }
}
