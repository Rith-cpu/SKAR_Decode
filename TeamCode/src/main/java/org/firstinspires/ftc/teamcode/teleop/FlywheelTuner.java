package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "FlywheelTuner (test)")
@Disabled

public class FlywheelTuner extends OpMode {

    public DcMotorEx Lshooter,Rshooter;
    public Servo lsServo;

    public double highVelocity = 1500;
    public double lowVelocity = 0.0;
    public double accVelocity = 0.0;

    public double hoodPos = 0.0;

    double currTargetvelocity = accVelocity;

    double F = 14.620;
    double P = 170;

    double[] stepSizesVel = {100.0, 50.0, 10.0, 1.0};
    double[] stepSizesHood = { 0.1, 0.01 ,0.001, 0.0001};


    int stepIndex = 1;

    @Override
    public void init() {
        lsServo = hardwareMap.get(Servo.class, "lshood");

        Lshooter = hardwareMap.get(DcMotorEx.class, "LS");
        Rshooter = hardwareMap.get(DcMotorEx.class, "RS");
        Lshooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rshooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rshooter.setDirection(DcMotorEx.Direction.REVERSE);


        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        Lshooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        Rshooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);

    }

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()){
            if (currTargetvelocity == accVelocity){
                currTargetvelocity = lowVelocity;
            } else { currTargetvelocity = accVelocity;}
        }

        if (gamepad1.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizesVel.length;
        }
        if (gamepad1.dpadLeftWasPressed()){
            accVelocity -= stepSizesVel[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()){
            accVelocity += stepSizesVel[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()){
            hoodPos -= stepSizesHood[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()){
            hoodPos += stepSizesHood[stepIndex];
        }



        Lshooter.setVelocity(currTargetvelocity);
        Rshooter.setVelocity(currTargetvelocity);
        lsServo.setPosition(hoodPos);


        double currVelocity = Lshooter.getVelocity();
        double error = currTargetvelocity - currVelocity;

        telemetry.addData("Target Velocity", currTargetvelocity);
        telemetry.addData("Curent velocity", "%.2f", currVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addData("Hood Posisiton", "%.4f (Dpad U/D)", hoodPos);
        telemetry.addLine("---------------------------------------");
        telemetry.addData("Step Size Velocity", "%.4f", stepSizesVel[stepIndex]);
        telemetry.addData("Step Size Hood", "%.4f", stepSizesHood[stepIndex]);



    }

}
