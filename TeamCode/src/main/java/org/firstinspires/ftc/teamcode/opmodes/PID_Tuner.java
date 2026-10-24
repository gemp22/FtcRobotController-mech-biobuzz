package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Disabled
@TeleOp(name="Teleop Name", group="Main")
public class PID_Tuner extends OpMode {

    private Intake intake;
    private Drivetrain drivetrain;
    @Override
    public void init() {

    }

    @Override
    public void loop() {
        handleDriverControls();
        handleOperatorControls();
        updateSubsystems();
    }

    public void handleDriverControls() {

    }

    public void handleOperatorControls() {

    }

    public void updateSubsystems() {
        intake.update();
        drivetrain.update();
    }
}
