package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.LimelightDistance;
import org.firstinspires.ftc.teamcode.teleop.TSHI;
import org.firstinspires.ftc.teamcode.teleop.TurretTesting;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous(group = "Red")
public class AutoRedClose extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        TSHI tshi = new TSHI(hardwareMap);
        TurretTesting tt = new TurretTesting(hardwareMap);
        LimelightDistance ld = new LimelightDistance(hardwareMap);
        Double distance = ld.getDistanceIfValid();

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {

            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(51, 50, Math.toRadians(225)));

            tshi.AutoIntakeandTransfer("transdown");
            tshi.AutoLS(0.1);
           // tshi.AutoTurret(230);

            tt.setPipeline(0);
            tt.getLimelightStatus();

            waitForStart();

            shootPreloads(drive, tshi, tt, ld);

            PickUpBALLS(drive, tshi, tt, ld);

            ShootSecond(drive, tshi, tt, ld);

        }

    }
    private void shootPreloads(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos1 = new Pose2d(51, 50, Math.toRadians(225));

        Double distance = ld.getDistanceIfValid();
        //tt.updateLL();   // poll Limelight so telemetry stays live
        // tt.runLL();      // always auto-align the turret
        int count = 0;


        while (count == 0) {
            tshi.AutoShooter("start",0.47);
            Actions.runBlocking(
                    drive.actionBuilder(pos1)
                            .strafeTo(new Vector2d(30, 29), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-90, 90))
                            .build());
            count++;
        }

        while (count == 0 || count == 1) {
            // // // // FIRST BALL // // // //
            sleep(2200);
            tshi.AutoIntakeandTransfer("transup");
            sleep(300);
            // // // // SECOND BALL // // // //
            tshi.AutoIntakeandTransfer("transdown");
            sleep(100);
            tshi.AutoIntakeandTransfer("intakerun");
            sleep(900);
            tshi.AutoIntakeandTransfer("intakestop");
            sleep(500);
            tshi.AutoIntakeandTransfer("transup");
            sleep(500);
            // // // // THIRD BALL // // //
            tshi.AutoIntakeandTransfer("transdown");
            sleep(900);
            tshi.AutoIntakeandTransfer("intakerun");
            sleep(2100);
            tshi.AutoIntakeandTransfer("transup");
            sleep(600);
            // // // // STOPS FUNCTIONS // // // //
            tshi.AutoIntakeandTransfer("transdown");
            tshi.AutoIntakeandTransfer("intakestop");
            count++;
        }


    }
    private void PickUpBALLS(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld){
        Pose2d pos1 = new Pose2d(30, 29, Math.toRadians(225));
        int count = 2;


        while (count == 2){
        Actions.runBlocking(
                drive.actionBuilder(pos1)
                        .strafeToLinearHeading(new Vector2d(32, -5),Math.toRadians(8))
                        .build());
        count++;
    }
        Pose2d pos2 = new Pose2d(new Vector2d(32, -5),Math.toRadians(8));
        tshi.AutoIntakeandTransfer("intakerun");
        Actions.runBlocking(
                drive.actionBuilder(pos2)
                        .strafeTo(new Vector2d(64, -3),new TranslationalVelConstraint(20), new ProfileAccelConstraint(-20, 20))
                        .build());
        tshi.AutoIntakeandTransfer("intakestop");
    }
    private void ShootSecond(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld){

        Pose2d pos1 = new Pose2d(new Vector2d(65, -3),Math.toRadians(8));

        Actions.runBlocking(
                drive.actionBuilder(pos1)
                        .strafeToSplineHeading(new Vector2d(30, 29),Math.toRadians(225))
                        .build());




        // // // // FIRST BALL // // // //
        sleep(2200);
        tshi.AutoIntakeandTransfer("transup");
        sleep(300);
        // // // // SECOND BALL // // // //
        tshi.AutoIntakeandTransfer("transdown");
        sleep(100);
        tshi.AutoIntakeandTransfer("intakerun");
        sleep(900);
        tshi.AutoIntakeandTransfer("intakestop");
        sleep(500);
        tshi.AutoIntakeandTransfer("transup");
        sleep(500);
        // // // // THIRD BALL // // //
        tshi.AutoIntakeandTransfer("transdown");
        sleep(900);
        tshi.AutoIntakeandTransfer("intakerun");
        sleep(2100);
        tshi.AutoIntakeandTransfer("transup");
        sleep(600);
        // // // // STOPS FUNCTIONS // // // //
        tshi.AutoIntakeandTransfer("transdown");
        tshi.AutoTurret(0);
        tshi.AutoIntakeandTransfer("intakestop");
        tshi.AutoShooter("stop", 0);


        Pose2d pos2 = new Pose2d(new Vector2d(30, 29),Math.toRadians(225));

        Actions.runBlocking(
                drive.actionBuilder(pos2)
                        .strafeTo(new Vector2d(50, 30))
                        .build());
    }



}


