package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "field relative")
public class FieldRelativeOpMode extends OpMode {

    FieldRelativeDrive frdrive = new FieldRelativeDrive(); // gets information from FieldRelativeDrive
    double forward, strafe, rotate; // variables later used as arguments for driveFieldRelative

    @Override
    public void init() {
        frdrive.FieldRelativeInit(hardwareMap); // Initializes hardware from FieldRelativeDrive
    }

    @Override
    public void loop() {
        forward = -gamepad1.left_stick_y; // setting each movement type to each button movement
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        frdrive.driveFieldRelative(forward, strafe, rotate); // variables above fill arguments from driveFieldRelative
    }
}
