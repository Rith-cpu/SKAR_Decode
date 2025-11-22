package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class TSHI {
    DcMotor Lshooter, Rshooter, turretMotor, intake;
    Servo lsservo, transfer;

    // Boot/transfer kicker variables
    private boolean isKicking = false;
    private long kickStartTime = 0;

    // Your servo
    private double transferRestPos = 0.12;   // resting/home position
    private double transferKickPos = 0.65;   // pushing forward position
    private long transferHoldMs = 250;       // time to hold forward (ms)

   public TSHI(HardwareMap Init) {
       turretMotor = Init.get(DcMotor.class, "trrt");
       Lshooter = Init.get(DcMotor.class, "LS");
       Rshooter = Init.get(DcMotor.class, "RS");
       intake = Init.get(DcMotor.class, "intake");
       lsservo = Init.get(Servo.class, "lshood");
       transfer = Init.get(Servo.class,"trans");

       lsservo.setPosition(0);
       transfer.setPosition(0.5);
       turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

   }
   public double getpos(){
       double tpos = turretMotor.getCurrentPosition();
       return tpos;
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
       if (gamepad.dpad_up) {
           transfer.setPosition(0.0);
       }
       if (gamepad.dpad_down){
           transfer.setPosition(0.5);
       }

       /*if (gamepad.dpad_left) {
           transfer.setPosition(0.5);
       }*/

       if (gamepad.x){
           intake.setPower(-1);
       }
       intake.setPower(0);

       double intakePower = intake.getPower();
       /*if ((intakePower >= -1.0 && intakePower <= -0.5) && gamepad.x) {
           intake.setPower(0.0);
       } else if ((intakePower > -0.5 && intakePower <= 0.0) && gamepad.x) {
           intake.setPower(-1.0);
       }*/


   }
   public void ShooterActions(Gamepad gamepad) {
       Lshooter.setPower(gamepad.right_trigger);
       Rshooter.setPower(-gamepad.right_trigger);

       /*if (gamepad.right_bumper) {
           turretMotor.setPower(-0.67);
       } else if (gamepad.left_bumper) {
           turretMotor.setPower(0.67);
       } else {
           turretMotor.setPower(0);
       }*/

   }
    public void TurretJoystickControl(Gamepad gamepad) {
        double lx = gamepad.left_stick_x;
        double ly = gamepad.left_stick_y;

        double deadzone = 0.15;

        if (Math.hypot(lx, ly) > deadzone) {

            double rawDeg = Math.toDegrees(Math.atan2(lx, -ly));
            double targetDeg = (rawDeg < 0) ? rawDeg + 360 : rawDeg;

            final int TICKS_PER_REV = 6090;
            double ticksPerDeg = TICKS_PER_REV / 360.0;
            int targetTicksMod = (int) Math.round(targetDeg * ticksPerDeg); // 0..TPR

            int currentTicks = turretMotor.getCurrentPosition();
            int currentMod = currentTicks % TICKS_PER_REV;
            if (currentMod < 0) currentMod += TICKS_PER_REV; // normalize to 0..TPR-1

            int delta = targetTicksMod - currentMod; // desired move within one rev
            // Wrap to shortest path
            if (delta > TICKS_PER_REV / 2) {
                delta -= TICKS_PER_REV;
            } else if (delta < -TICKS_PER_REV / 2) {
                delta += TICKS_PER_REV;
            }

            int finalTarget = currentTicks + delta; // absolute encoder target

            turretMotor.setTargetPosition(finalTarget);
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            turretMotor.setPower(0.7);

        } else {
            turretMotor.setPower(0);
        }
    }
    /*public void startTransferKick() {
        if (!isKicking) {
            transfer.setPosition(transferKickPos);
            kickStartTime = System.currentTimeMillis();
            isKicking = true;
        }
    }

    public void updateTransferKick() {
        if (isKicking) {
            if (System.currentTimeMillis() - kickStartTime >= transferHoldMs) {
                transfer.setPosition(transferRestPos);
                isKicking = false;
            }
        }
    }

    public void TransferKick(Gamepad gamepad){
        if (gamepad.right_trigger > 0.5) {
            startTransferKick();
        }
        updateTransferKick();
    }*/

}
