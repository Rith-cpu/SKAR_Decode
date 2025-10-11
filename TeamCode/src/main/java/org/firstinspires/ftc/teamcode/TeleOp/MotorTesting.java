package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MotorTest")
public class MotorTesting extends OpMode {

    DcMotor testMotorup;
    DcMotor testMotordown;
    @Override
    public void init() {
        testMotorup = hardwareMap.get(DcMotor.class,"test");
        testMotordown = hardwareMap.get(DcMotor.class,"test2");
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper){
            teewstMotorup.setPower(-1);
            testMotordown.setPower(-1);

        }
        //if (gamepad1.left_bumper){
        //testMotorup.setPower(-1);
        //}
        testMotorup.setPower(0);
        testMotordown.setPower(0);

    }
}
