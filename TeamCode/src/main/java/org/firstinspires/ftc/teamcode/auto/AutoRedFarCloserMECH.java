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


@Autonomous(name = "AutoRedFarMECH",group = "Red")
public class AutoRedFarCloserMECH extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        TSHI tshi = new TSHI(hardwareMap);
        TurretTesting tt = new TurretTesting(hardwareMap);
        LimelightDistance ld = new LimelightDistance(hardwareMap);

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {

            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(14, -70, Math.toRadians(0)));

            tshi.AutoLS(0.9);
            tshi.AutoTurret(-440);

            tt.setPipeline(0);
            tt.getLimelightStatus();

            //tt.updateLL();   // poll Limelight so telemetry stays live
            //tt.runLL();

            waitForStart();

            shootPreloads(drive, tshi, tt, ld);

            intakeBalls(drive, tshi, tt, ld);

            shoot(drive, tshi, tt, ld);

            intakeBalls2(drive, tshi, tt, ld);

            //shootAgain(drive, tshi, tt, ld);

        }

    }
    private void shootPreloads(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos = new Pose2d(14, -70, Math.toRadians(0));

        Double distance = ld.getDistanceIfValid();
        // always auto-align the turret

        tshi.AutoShooter("start", 1480.14642);

        /*Actions.runBlocking(
                drive.actionBuilder(pos)
                        .strafeTo(new Vector2d(14, -60),new TranslationalVelConstraint(100), new ProfileAccelConstraint(-90, 90))
                        .build());*/

        // SHOOTING SEQUENCE //
        sleep(1000);
        tshi.AutoIntakeandShoot("shoot", 1);
        sleep(2500);
        tshi.AutoShooter("stop",0);
        // // // // // // // //
    }

    private void intakeBalls(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos = new Pose2d(14, -70, Math.toRadians(0));

        tshi.AutoIntakeandShoot("intake", 1);

        Actions.runBlocking(
                drive.actionBuilder(pos)
                        .strafeToLinearHeading(new Vector2d(45, -40), Math.toRadians(10), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-90, 90))
                        .build());

        sleep(300);

        Pose2d pos2 = new Pose2d(45, -40, Math.toRadians(10));

        Actions.runBlocking(
                drive.actionBuilder(pos2)
                        .strafeTo(new Vector2d(60, -40), new TranslationalVelConstraint(10), new ProfileAccelConstraint(-90, 90))
                        .build());

        tshi.AutoIntakeandShoot("stop", 0);


        //Pose2d pos3 = new Pose2d(60, -40, 0);

        /*Actions.runBlocking(
                drive.actionBuilder(pos3)
                        .strafeToLinearHeading(new Vector2d(14, -40), 20)
                        .build());*/

        //sleep(300);
    }

    private void shoot(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos = new Pose2d(60, -40, Math.toRadians(10));

        Double distance = ld.getDistanceIfValid();
        tshi.AutoTurret(-275);
        tshi.AutoShooter("start", 1510.14642);

        Actions.runBlocking(
                drive.actionBuilder(pos)
                        .strafeToLinearHeading(new Vector2d(14, -57), Math.toRadians(20))
                        .build());

        // SHOOTING SEQUENCE //
        sleep(300);
        tshi.AutoIntakeandShoot("shoot", 1);
        sleep(2500);
        tshi.AutoShooter("stop",0);
        // // // // // // // //

    }

    private void intakeBalls2(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos = new Pose2d(14, -57, Math.toRadians(20));

        tshi.AutoIntakeandShoot("intake", 1);

        Actions.runBlocking(
                drive.actionBuilder(pos)
                        .strafeToLinearHeading(new Vector2d(30, -57), Math.toRadians(0))
                        .build());

        tshi.AutoTurret(0);

        sleep(500);

        /*Pose2d pos2 = new Pose2d(40, -16, Math.toRadians(0));

        Actions.runBlocking(
                drive.actionBuilder(pos2)
                        .strafeToLinearHeading(new Vector2d(58, -17), 0, new TranslationalVelConstraint(70))
                        .build());

        tshi.AutoIntakeandShoot("stop", 0);

        Pose2d pos3 = new Pose2d(60, -18, Math.toRadians(0));

        Actions.runBlocking(
                drive.actionBuilder(pos3)
                        .strafeTo(new Vector2d(14, -18), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-90,90))
                        .build());


        sleep(300);*/

    }

    private void shootAgain(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos = new Pose2d(58, -17, Math.toRadians(0));


        Double distance = ld.getDistanceIfValid();
        tshi.AutoTurret(-465);
        tshi.AutoShooter("start", 1294.64553); // if 14 put 59

        Actions.runBlocking(
                drive.actionBuilder(pos)
                        .strafeToLinearHeading(new Vector2d(10, -5), Math.toRadians(-20))
                        .build());

        // SHOOTING SEQUENCE //
        sleep(300);
        tshi.AutoIntakeandShoot("shoot", 1);
        sleep(2500);
        tshi.AutoShooter("stop",0);
        // // // // // // // //


        Pose2d pos2 = new Pose2d(10, -5, Math.toRadians(-20));

        Actions.runBlocking(
                drive.actionBuilder(pos2)
                        .strafeToLinearHeading(new Vector2d(40, -5), 0)
                        .build());

        tshi.AutoTurret(-20);
        sleep(1000);


    }
}