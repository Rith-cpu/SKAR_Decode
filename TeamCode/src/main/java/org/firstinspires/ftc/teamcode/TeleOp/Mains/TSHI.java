package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TSHI {
    DcMotor Lshooter, Rshooter, turretMotor, intake;
    Servo lsservo, transfer;

   public TSHI(HardwareMap Init) {
       turretMotor = Init.get(DcMotor.class, "trrt");
       Lshooter = Init.get(DcMotor.class, "LS");
       Rshooter = Init.get(DcMotor.class, "RS");
       intake = Init.get(DcMotor.class, "intake");
       lsservo = Init.get(Servo.class, "lshood");
       transfer = Init.get(Servo.class,"trans");

       lsservo.setPosition(0);
       transfer.setPosition(0);
       turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

   }

   public void LSActions(Gamepad gamepad) {
       if (gamepad.y) {
           lsservo.setPosition(0.89);
       }
       if (gamepad.a) {
           lsservo.setPosition(0.12);
       }
       if (gamepad.b) {
           lsservo.setPosition(0.5);
       }
   }
   public void IntakeandTransfer(Gamepad gamepad) {
       if (gamepad.x) {
           intake.setPower(-1);
       }
       if (gamepad.right_stick_button) {
           transfer.setPosition(1.0);
       }
       intake.setPower(0);
       transfer.setPosition(0);
   }
   public void TurretandShooterActions(Gamepad gamepad) {
       Lshooter.setPower(gamepad.right_stick_y);
       Rshooter.setPower(-gamepad.right_stick_y);

       if (gamepad.right_bumper) {
           turretMotor.setPower(-0.67);
       } else if (gamepad.left_bumper) {
           turretMotor.setPower(0.67);
       } else {
           turretMotor.setPower(0);
       }

   }
}
