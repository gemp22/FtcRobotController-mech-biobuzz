package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.util.BallColor;
import org.firstinspires.ftc.teamcode.util.HuskyLensUtil;
import org.firstinspires.ftc.teamcode.util.FlyWheel;

import java.util.Locale;


/**
 * The shooter subsystem is responsible for collecting balls from the field,
 * transporting them internally, detecting their color, and managing the gates
 * that release them to the shooter.
 */
public class Shooter {

    // --- Constants ---
    public final FlyWheel flyWheel1;
    public final FlyWheel flyWheel2;
    private static final double shooter_ROLLER_SPEED = 1.0;

    private static final double shooter_ROLLER_SPEED_FORWARD = 1.0;
    private static final double shooter_ROLLER_SPEED_BACKWARD = -1.0;

    private static final double CRSERVO_FORWARD_POWER = 1.0;
    private static final double CRSERVO_REVERSE_POWER = -1.0;
    private static final double CRSERVO_STOP_POWER = 0.0;
    private static final double ROLLER_BED_SPEED_INCREMENT = 0.1;
    private static final long GATE_OPEN_DURATION_MS = 3000;

    // --- Constants for Shake Mode ---
    private static final long ROLLER_BED_SHAKE_FORWARD_MS = 300;
    private static final long ROLLER_BED_SHAKE_REVERSE_MS = 400;
    private static final double ROLLER_BED_SHAKE_SPEED = 0.7;

    private boolean isCameraSensingEnabled = false;

    private static final double LEFT_GATE_OPEN_POS = 1;
    private static final double LEFT_GATE_CLOSE_POS = 0.279;
    private static final double MID_GATE_OPEN_POS = 0;
    private static final double MID_GATE_CLOSE_POS = .743;
    private static final double RIGHT_GATE_OPEN_POS = 0;
    private static final double RIGHT_GATE_CLOSE_POS = 0.74;


    // --- enum for the shooter states ---
    public enum shooterState {
        OFF,
        FORWARD,
        REVERSE
    }

    public enum FeederPosition {
        UP,
        DOWN
    }

    // --- Hardware ---
    private DcMotorEx motor1;
    private DcMotorEx motor2;
    // --- State ---


    private shooterState currentshooterState = shooterState.OFF;
    private shooterState previousshooterState = shooterState.OFF;

    private FeederPosition currentFeederPosition = FeederPosition.DOWN;
    private double lastFeederUpPosition = 0;

    private double forwardRPM = 2200.0;


    // --- Flag to control color sensor updates ---private boolean isColorSensingEnabled = false;

    // --- Flag to control HuskyLens camera updates ---

    // --- State variables to track last set power ---
//    private double lastshooterRollerPower = -999;

    public Shooter(RobotHardware robot) {
        this.motor1 = robot.shooterMotor1;
        this.motor2 = robot.shooterMotor2;

        // NEW: Cast motors to DcMotorEx to support velocity-based PIDF control
        this.flyWheel1 = new FlyWheel(motor1, FlyWheel.FlyWheelNum.ONE);
        this.flyWheel2 = new FlyWheel(motor2, FlyWheel.FlyWheelNum.TWO);

        // NEW: Set PIDF coefficients (kP, kI, kD, kF)
        // 0.5 kF provides ~50% baseline power for the 6000 RPM motors
//        this.rollerController.setPIDFCoefficients(0.01, 0.0, 0.0, 0.5);

        /* --- OLD CODE (PRE-PIDF) ---
        this.shooterRoller.setDirection(DcMotor.Direction.FORWARD);
        this.shooterRoller2.setDirection(DcMotor.Direction.FORWARD);
        this.shooterRoller.setPower(0);
        this.shooterRoller2.setPower(0);
        */
    }

    // --- High-Level Control Methods ---







    /**
     * Directly sets the state of the main shooter roller.
     * @param on True to turn the roller on, false to turn it off.
     */





    /**
     * This method handles state updates for the shooter roller.
     * Uses the PIDF controller for forward motion and manual power for reverse.
     */
    public void update() {


        flyWheel1.update();
        flyWheel2.update();

    }


    /**
     * Stops all motors and resets the subsystem state.
     */
    public void stop() {
        // NEW: Stop the PID controller (handles resetting motor power to 0)
        flyWheel1.stop();
        flyWheel2.stop();


    }





    /**
     * Sets the target RPM for the FORWARD shooter state.
     * @param rpm The target RPM.
     */
    public void setForwardRPM(double rpm) {
        this.forwardRPM = rpm;
        // If we are currently in FORWARD state, update the controller immediately
//        if (currentshooterState == shooterState.FORWARD) {
        flyWheel1.setRPM(forwardRPM);
        flyWheel2.setRPM(forwardRPM);
//        }
    }

    public void setForwardPower(double power) {
        flyWheel1.setPower(power);
        flyWheel2.setPower(power);
    }

    /**
     * Gets the current target RPM for the FORWARD shooter state.
     * @return The target RPM.
     */
    public double getForwardRPM() {
        return forwardRPM;
    }

    /**
     * Enables or disables debugging for the shooter roller PIDF controller.
     * @param enable True to enable, false to disable.
     */
    public void setDebug(boolean enable) {
        flyWheel1.setDebug(enable);
        flyWheel2.setDebug(enable);
    }

    /**
     * Enables debug plotting for a specific color sensor.
     * Disables debugging on all others.
     * @param sensorIndex 0 for Left, 1 for Mid, 2 for Right. Use -1 to disable all.
     */
    public void setDebug(int sensorIndex) {
    }


    // --- Private Helper Methods ---

    public boolean isDebugEnabled() {
        return flyWheel1.isDebugEnabled() || flyWheel2.isDebugEnabled();
    }
    /**
     * Manages the state machine for the roller bed shake mode.
     * Should only be called from update() when isShakeModeOn is true.
     */


    // --- Telemetry Methods ---







}


