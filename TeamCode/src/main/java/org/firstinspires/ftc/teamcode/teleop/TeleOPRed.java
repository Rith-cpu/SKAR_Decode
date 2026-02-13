package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.prism.GoBildaPrismDriver;
//import org.firstinspires.ftc.teamcode.auto.Trig;


@TeleOp(name = "TeleOP -  Red - 1P" )
public class TeleOPRed extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap,new Pose2d(0, 0, 0));
        TSHI tshi = new TSHI(hardwareMap);
        Drivetrain driveTrainControl = new Drivetrain(hardwareMap);
        TurretTesting tt = new TurretTesting(hardwareMap);
        LimelightDistance distance = new LimelightDistance(hardwareMap);
        tshi.prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1);
        //Trig trig = new Trig(hardwareMap, drive);


        tt.setPipeline(0);
        tt.getLimelightStatus();
        // Optional: tune these later; start with defaults in TurretTesting
        // tt.setGains(0.04, 1.0, 0.7);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            Double llDistance = distance.getDistanceIfValid();

            tt.updateLL();   // poll Limelight so telemetry stays live
            tt.runLL();      // always auto-align the turret
            // If auto-aim is not running this loop, the last Limelight values may be stale.
            distance.getDistance(llDistance);
            //trig.autoAlignTurret();
            //use for testing odometry
            // This keeps telemetry truthful.
            telemetry.addData("LL status", tt.getLimelightStatus());
            telemetry.addData("LL valid", tt.hasValidTarget());
            //telemetry.addData("x", trig.getCurrPosX());
            //telemetry.addData("y", trig.getCurrPosY() );
            //telemetry.addData("heading", trig.turretAngle());
            //telemetry.addData("Distance based off odometry", trig.distance());




            if("INVALID".equalsIgnoreCase(tt.getLimelightStatus())){
                telemetry.addData("Testing invalid status", tshi.Lshooter.getVelocity());
                telemetry.update();
                tshi.LSActionsNotValidMode(100.00);
                tshi.ShooterActionsNotValidMode(gamepad1);
            }else{
                tshi.LSActions(distance);
                telemetry.addData("LL Distance", llDistance);
                if (llDistance < 50){
                    telemetry.addData("LowLow speed", distance.getLowLowPowerIfValid(llDistance));

                }

                if (llDistance < 100 && llDistance >50 ){
                    telemetry.addData("Low speed", distance.getLowPowerIfValid(llDistance));
                }
                if (llDistance > 100){
                    telemetry.addData("High Speed", distance.getHighPowerIfValid(llDistance));
                }

            }

            if (llDistance == null) {
                tshi.turretMotor.setTargetPosition(0);
                tshi.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                tshi.turretMotor.setPower(0.7);

            }

            telemetry.addData("Shooter Speed (Motor)", tshi.Lshooter.getVelocity());
            telemetry.addData("LS Position", tshi.lsservo.getPosition());
            telemetry.addData("Turret Position", tshi.turretMotor.getCurrentPosition());
            /*telemetry.addLine("----------------------------------");
            telemetry.addLine("Flywheel Tuning Values");
            telemetry.addData("Target Velocity", tshi.accVelocity);
            telemetry.addData("Curent velocity",  tshi.Lshooter.getVelocity());
            telemetry.addData("Hood Posisiton",  tshi.hoodPos);
            telemetry.addLine("---------------------------------------");
            telemetry.addData("Step Size Velocity",  tshi.stepSizesVel[tshi.stepIndex]);
            telemetry.addData("Step Size Hood", tshi.stepSizesHood[tshi.stepIndex]);*/
            telemetry.addLine("----------------------------------");
            telemetry.addLine("Custom Flywheel Values");
            telemetry.addData("Current Selected Equation",  distance.equation[Math.abs(distance.stepIndex)]);
            telemetry.addData("Delta y-int Less than 50:", distance.b-973.16097);
            telemetry.addData("Delta y-int Less than 100 & Greater than 50:", distance.b50-968.67251);
            telemetry.addData("Delta y-int Greater than 100:", distance.b100-1061.85831);
            telemetry.update();



            driveTrainControl.Driving(gamepad1);
            tshi.ShooterActions(gamepad1,0.279,distance);
            tshi.Intake(gamepad1);
            //tshi.backup(gamepad1,0.279);
            //tshi.FlywheelTuning(gamepad1);
            distance.customFlyweelVel(gamepad1);


            //Tempprary manual control
            //turretShooterHoodIntake.TurretJoystickControl(gamepad2);
            //tt.runLL();



        }
    }
}
