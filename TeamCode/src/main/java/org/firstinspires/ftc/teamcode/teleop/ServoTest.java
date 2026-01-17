package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.roadrunner.Line;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoTest extends OpMode {
    CRServo s1,s2;

    DcMotor LF_motor,tm1,tm2;

    @Override
    public void init() {
        s1 = hardwareMap.get(CRServo.class, "s1");
        s2 = hardwareMap.get(CRServo.class, "s2");
        LF_motor = hardwareMap.get(DcMotor.class, "m1");
        tm1 = hardwareMap.get(DcMotor.class, "tm1");
        tm2 = hardwareMap.get(DcMotor.class, "tm2");

        tm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }


    @Override
    public void loop() {
        double LF_power = 0;
        if(gamepad1.left_stick_x !=0){
            LF_power += gamepad1.left_stick_x;
        }
        if (gamepad1.a) {
            s1.setPower(1.0);
            s2.setPower(-1.0);

        }
        if(gamepad1.right_stick_y !=0){
            LF_power += gamepad1.left_stick_x;
            tm1.setPower(1.0);
            tm2.setPower(-1.0);
        }

        LF_motor.setPower(LF_power);
    }
}
