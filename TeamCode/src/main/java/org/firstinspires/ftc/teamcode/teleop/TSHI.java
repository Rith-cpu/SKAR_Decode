package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.prism.GoBildaPrismDriver.Artboard;
import org.firstinspires.ftc.teamcode.prism.GoBildaPrismDriver;


public class TSHI {
    DcMotor turretMotor, intake;
    DcMotorEx Lshooter, Rshooter;
    Servo lsservo, transfer ;
    GoBildaPrismDriver prism;

    //   LimelightDistance distance;
    // Boot/transfer kicker variables
    private boolean isKicking = false;
    private long kickStartTime = 0;

    // Your servo
    private double transferRestPos = 0.12;   // resting/home position
    private double transferKickPos = 0.65;   // pushing forward position
    private long transferHoldMs = 250;       // time to hold forward (ms)
    private double speed = 0.0;

    double P = 170;
    double F = 14.620;
    public TSHI(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotor.class, "trrt");
        Lshooter = hardwareMap.get(DcMotorEx.class, "LS");
        Rshooter = hardwareMap.get(DcMotorEx.class, "RS");
        intake = hardwareMap.get(DcMotor.class, "intake");
        lsservo = hardwareMap.get(Servo.class, "lshood");
        transfer = hardwareMap.get(Servo.class,"trans");
        prism = hardwareMap.get(GoBildaPrismDriver.class,"LED");


        //    distance= new LimelightDistance(hardwareMap);
        lsservo.setPosition(0);
        transfer.setPosition(0.5);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Lshooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rshooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rshooter.setDirection(DcMotorEx.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);

        Lshooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        Rshooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);




        prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_0);

    }
    public double getpos(){
        double tpos = turretMotor.getCurrentPosition();
        return tpos;
    }

    public void LSActions(Gamepad gamepad, LimelightDistance limelightDistance) {

        if(limelightDistance.getDistanceIfValid()!=null){
            Double distance = limelightDistance.getDistanceIfValid();

            if (distance >= 100.0){
                lsservo.setPosition(0.9);
                speed = limelightDistance.getHighPowerIfValid(distance);
            }
            if (distance < 50.0){
                lsservo.setPosition(0.1);
                speed = limelightDistance.getLowLowPowerIfValid(distance);
            }
            if (distance < 100.00 && distance >= 50.0){
                lsservo.setPosition(0.5);
                speed = limelightDistance.getLowPowerIfValid(distance);
            }
            if (gamepad.y) {
                lsservo.setPosition(0.9);
            }
            if (gamepad.a) {
                lsservo.setPosition(0.1);
            }
            if (gamepad.b) {
                lsservo.setPosition(0.5);
            }

        }

    }
    public void IntakeandTransfer(Gamepad gamepad) {
        if (gamepad.dpad_up) {
            transfer.setPosition(0.2);
        }
        if (gamepad.dpad_down){
            transfer.setPosition(0.5);
        }


        if (gamepad.right_bumper){
            intake.setPower(-1);
        }
        if (gamepad.left_bumper){
            intake.setPower(1);
        }
        intake.setPower(0);



    }
    public void ShooterActions(Gamepad gamepad) {
        prism.loadAnimationsFromArtboard(Artboard.ARTBOARD_0);
      /* Lshooter.setPower(gamepad.right_trigger);
       Rshooter.setPower(-gamepad.right_trigger);*/
        if (gamepad.right_trigger > 0) {
            Lshooter.setPower(speed);
            Rshooter.setPower(-speed);

        }
        if (gamepad.right_trigger == 0){
            Lshooter.setPower(0);
            Rshooter.setPower(0);

        }
        if (gamepad.touchpad){
            for(int i = 0; i<3; i++){
                intake.setPower(0);
                Lshooter.setPower(0.8); Rshooter.setPower(-0.8); //arbitray value

                transfer.setPosition(0.2);
                transfer.setPosition(0.5);
                intake.setPower(-1);
            }

        }




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


    public void AutoIntakeandTransfer(String pos) {
        pos.toLowerCase();

        if (pos == "transdown") {
            transfer.setPosition(0.5);
        }
        if (pos == "transup") {
            transfer.setPosition(0.2);
        }

        if (pos == "intakerun") {
            intake.setPower(-1);
        }
        if (pos == "intakestop") {
            intake.setPower(0);
        }
        if (pos == "intakereverse") {
            intake.setPower(1);
        }
        if (pos == "intakeslow") {
            intake.setPower(.6);
        }

    }
    public void AutoShooter(String status, double sped) {
        status.toLowerCase();
        if (status == "start") {
            Lshooter.setPower(sped);
            Rshooter.setPower(-sped);
        }
        if (status == "stop") {
            Lshooter.setPower(0);
            Rshooter.setPower(0);
        }

    }

    public void AutoLS(double pos) {
        lsservo.setPosition(pos);
    }
    public void AutoTurret(int pos) {
        turretMotor.setTargetPosition(pos);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.7);
    }

}
