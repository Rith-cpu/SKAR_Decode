package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "CRServoTest")
@Disabled
public class CRServoTest extends OpMode {
    CRServo servo;
    @Override
    public void init() {
        servo = hardwareMap.get(CRServo.class,"ser");
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            servo.setPower(1.0);
        }
        servo.setPower(0);
    }
}
