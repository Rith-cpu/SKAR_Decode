package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
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
public class AutoRedClose12 extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        TSHI tshi = new TSHI(hardwareMap);
        TurretTesting tt = new TurretTesting(hardwareMap);
        LimelightDistance ld = new LimelightDistance(hardwareMap);

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {

            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(50, 55, Math.toRadians(45)));

            tshi.AutoLS(0.5);
            tshi.AutoTurret(-275);

            tt.setPipeline(0);
            tt.getLimelightStatus();

            waitForStart();

            shootPreloads(drive, tshi, tt, ld);

            intakeBalls1(drive, tshi, tt, ld);

            shootSecondLoad(drive, tshi, tt, ld);

            IntakeSecondRow(drive, tshi, tt, ld);

            ShootThirdLoad(drive, tshi, tt, ld);

            IntakeThirdLine(drive, tshi, tt, ld);

            ShootFourthLoad(drive, tshi, tt, ld);

        }

    }

    private void shootPreloads(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos = new Pose2d(50, 55, Math.toRadians(45));
        int count = 1;
        Double distance = ld.getDistanceIfValid();
        //tt.updateLL();   // poll Limelight so telemetry stays live
        // tt.runLL();      // always auto-align the turret
        tshi.AutoShooter("start", 1200);

        Actions.runBlocking(
                drive.actionBuilder(pos)
                        .strafeToSplineHeading(new Vector2d(18, 20),Math.toRadians(10), new TranslationalVelConstraint(100))
                        .build());


        tshi.AutoIntakeandShoot("shoot", 1);
        sleep(1800);
        tshi.AutoShooter("stop", 0);

    }

    private void intakeBalls1(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos = new Pose2d(18, 20, Math.toRadians(10));


        tshi.AutoIntakeandShoot("intake", 1.0);


        Actions.runBlocking(
                drive.actionBuilder(pos)
                        .strafeTo(new Vector2d(50, 20), new TranslationalVelConstraint(90))
                        .strafeToSplineHeading(new Vector2d(70,11), Math.toRadians(90))
                        .build());


        tshi.AutoIntakeandShoot("stop", 0);

    }

    private void shootSecondLoad(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos = new Pose2d(70, 11, Math.toRadians(90));
        int count = 1;
        Double distance = ld.getDistanceIfValid();
        //tt.updateLL();   // poll Limelight so telemetry stays live
        // tt.runLL();      // always auto-align the turret
        tshi.AutoTurret(-320);
        tshi.AutoShooter("start", 1230);


        Actions.runBlocking(
                drive.actionBuilder(pos)
                        .strafeToSplineHeading(new Vector2d(27, 20), Math.toRadians(10),new TranslationalVelConstraint(100))
                        .build());

        tshi.AutoIntakeandShoot("shoot", 1);

        sleep(1600);
        tshi.AutoShooter("stop", 0);

    }
    private void IntakeSecondRow(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos = new Pose2d(27, 20, Math.toRadians(10));

        tshi.AutoIntakeandShoot("intake", 1.0);


        Actions.runBlocking(
                drive.actionBuilder(pos)
                        .strafeToLinearHeading(new Vector2d(25, -5), Math.toRadians(10))
                        .strafeToLinearHeading(new Vector2d(57, -3), Math.toRadians(10),new TranslationalVelConstraint(40))
                        .build());


        tshi.AutoIntakeandShoot("stop", 0);
    }

    private void ShootThirdLoad(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos = new Pose2d(57, -3, Math.toRadians(10));
        int count = 1;
        Double distance = ld.getDistanceIfValid();

        tshi.AutoShooter("start", 1265);

        tshi.AutoTurret(-300);

            Actions.runBlocking(
                    drive.actionBuilder(pos)
                            .strafeTo(new Vector2d(13, 8))
                            .build());

        tshi.AutoIntakeandShoot("shoot", 1);

        sleep(1600);
        tshi.AutoShooter("stop", 0);


    }
    private void IntakeThirdLine (MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos = new Pose2d(13, 8, Math.toRadians(10));
        Double distance = ld.getDistanceIfValid();


        tshi.AutoIntakeandShoot("intake", 1.0);

            Actions.runBlocking(
                    drive.actionBuilder(pos)
                            .strafeTo(new Vector2d(35, -28),new TranslationalVelConstraint(100))
                            .strafeTo(new Vector2d(60, -28), new TranslationalVelConstraint(100))
                            .build());

        tshi.AutoIntakeandShoot("stop", 1.0);

    }
    private void ShootFourthLoad (MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos = new Pose2d(60, -28, Math.toRadians(10));
        Double distance = ld.getDistanceIfValid();


        tshi.AutoShooter("start", 1170);

        tshi.AutoTurret(0);


        Actions.runBlocking(
                drive.actionBuilder(pos)
                        .strafeToSplineHeading(new Vector2d(17, 33), Math.toRadians(40) ,new TranslationalVelConstraint(100))
                        .build());

        tshi.AutoIntakeandShoot("shoot", 1);

        sleep(1600);
        tshi.AutoShooter("stop", 0);



    }

}

