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
import org.opencv.video.TrackerNano_Params;

@Autonomous(name = "AutoRedClose", group = "Red")
public class AutoCloseRed9 extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        TSHI tshi = new TSHI(hardwareMap);
        TurretTesting tt = new TurretTesting(hardwareMap);
        LimelightDistance ld = new LimelightDistance(hardwareMap);

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {

            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(50, 55, Math.toRadians(45)));

            tshi.AutoLS(0.1);
            tshi.AutoTurret(-51);

            tt.setPipeline(0);
            tt.getLimelightStatus();

            waitForStart();

            shootPreloads(drive, tshi, tt, ld);

            intakeBalls1(drive, tshi, tt, ld);

            shootSecondLoad(drive, tshi, tt, ld);

            IntakeSecondRow(drive, tshi, tt, ld);

            ShootThirdLoad(drive, tshi, tt, ld);


        }

    }




    private void shootPreloads(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos = new Pose2d(50, 55, Math.toRadians(45));
        int count = 1;
        Double distance = ld.getDistanceIfValid();
        //tt.updateLL();   // poll Limelight so telemetry stays live
        // tt.runLL();      // always auto-align the turret
        tshi.AutoShooter("start", 1180);

        while (count == 1) {
            Actions.runBlocking(
                    drive.actionBuilder(pos)
                            .strafeTo(new Vector2d(28, 33),new TranslationalVelConstraint(100))
                            .build());
            count++;
        }
        while (count == 1 || count == 2) {
            tshi.AutoIntakeandShoot("shoot", 1);
            count++;}
        sleep(2200);
        tshi.AutoShooter("stop", 0);

    }

    private void intakeBalls1(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos = new Pose2d(28, 33, Math.toRadians(45));


        tshi.AutoIntakeandShoot("intake", 1.0);


        Actions.runBlocking(
                drive.actionBuilder(pos)
                        .strafeToLinearHeading(new Vector2d(23, 15), Math.toRadians(10))
                        .strafeTo(new Vector2d(53, 15), new TranslationalVelConstraint(90))

                        .build());


        tshi.AutoIntakeandShoot("stop", 0);

    }

    private void shootSecondLoad(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos = new Pose2d(53, 15, Math.toRadians(10));
        int count = 1;
        Double distance = ld.getDistanceIfValid();
        //tt.updateLL();   // poll Limelight so telemetry stays live
        // tt.runLL();      // always auto-align the turret
        tshi.AutoShooter("start", 1200);

        tshi.AutoTurret(-300);

        while (count == 1) {
            Actions.runBlocking(
                    drive.actionBuilder(pos)
                            .strafeTo(new Vector2d(20, 15))
                            .build());
            count++;
        }
        while (count == 1 || count == 2) {
            tshi.AutoIntakeandShoot("shoot", 1);
            count++;
        }
        sleep(2200);
        tshi.AutoShooter("stop", 0);

    }
    private void IntakeSecondRow(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos = new Pose2d(20, 15, Math.toRadians(10));

        tshi.AutoIntakeandShoot("intake", 1.0);


        Actions.runBlocking(
                drive.actionBuilder(pos)
                        .strafeToLinearHeading(new Vector2d(20, -10), Math.toRadians(10))
                        .strafeToLinearHeading(new Vector2d(66, -6), Math.toRadians(10),new TranslationalVelConstraint(50))
                        .build());


        tshi.AutoIntakeandShoot("stop", 0);
    }

    private void ShootThirdLoad(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos = new Pose2d(66, -6, Math.toRadians(10));
        int count = 1;
        Double distance = ld.getDistanceIfValid();

        tshi.AutoShooter("start", 1250);

        tshi.AutoTurret(-300);

        while (count == 1) {
            Actions.runBlocking(
                    drive.actionBuilder(pos)
                            .strafeTo(new Vector2d(13, 8))
                            .build());
            count++;
        }
        while (count == 1 || count == 2) {
            tshi.AutoIntakeandShoot("shoot", 1);
            count++;}
        sleep(2200);
        tshi.AutoShooter("stop", 0);
        sleep(500);
        tshi.AutoTurret(0);

        Pose2d pos1 = new Pose2d(13, 8, Math.toRadians(10));
        Actions.runBlocking(
                drive.actionBuilder(pos1)
                        .strafeTo(new Vector2d(27, 8))
                        .build());

    }
}

