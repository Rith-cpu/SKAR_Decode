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



@Autonomous(name = "AutoBlueFar",group = "Blue")
public class AutoBlueFarCloser extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        TSHI tshi = new TSHI(hardwareMap);
        TurretTesting tt = new TurretTesting(hardwareMap);
        LimelightDistance ld = new LimelightDistance(hardwareMap);



        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {

            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-14, -70, Math.toRadians(180)));

            tshi.AutoLS(0.9);
            tshi.AutoTurret(450);

            tt.setPipeline(1);
            tt.getLimelightStatus();

            waitForStart();

            shootPreloads(drive, tshi, tt, ld);

            intakeBalls(drive, tshi, tt, ld);

            shoot(drive, tshi, tt, ld);

            intakeBalls2(drive, tshi, tt, ld);

            shootAgain(drive, tshi, tt, ld);

        }

    }
    private void shootPreloads(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos = new Pose2d(-14, -70, Math.toRadians(180));

        Double distance = ld.getDistanceIfValid();
        //tt.updateLL();   // poll Limelight so telemetry stays live
        // tt.runLL();      // always auto-align the turret

        tshi.AutoShooter("start", ld.estimatePowerFromDistanceGreaterThan100(distance) - 10);

        /*Actions.runBlocking(
                drive.actionBuilder(pos)
                        .strafeTo(new Vector2d(-14, -60),new TranslationalVelConstraint(50), new ProfileAccelConstraint(-90, 90))
                        .build());*/

        // SHOOTING SEQUENCE //
        sleep(1200);
        tshi.AutoIntakeandShoot("shoot", 1);
        sleep(2500);
        tshi.AutoShooter("stop",0);
        // // // // // // // //
    }

    private void intakeBalls(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos = new Pose2d(-14, -70, Math.toRadians(180));

        tshi.AutoIntakeandShoot("intake", 1);

        Actions.runBlocking(
                drive.actionBuilder(pos)
                        .strafeTo(new Vector2d(-25, -45),new TranslationalVelConstraint(100), new ProfileAccelConstraint(-90, 90))
                        .build());

        sleep(500);

        Pose2d pos2 = new Pose2d(-25, -45, Math.toRadians(180));

        Actions.runBlocking(
                drive.actionBuilder(pos2)
                        .strafeTo(new Vector2d(-65, -45), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-20, 90))
                        .build());

        tshi.AutoIntakeandShoot("stop", 0);


        /*Pose2d pos3 = new Pose2d(-65, -43, Math.toRadians(180));

        Actions.runBlocking(
                drive.actionBuilder(pos3)
                        .strafeTo(new Vector2d(-14, -43), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-90,90))
                        .strafeTo(new Vector2d(-14, -57),new TranslationalVelConstraint(30), new ProfileAccelConstraint(-20, 90))
                        .build());

        sleep(300);*/
    }

    private void shoot(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos3 = new Pose2d(-65, -45, Math.toRadians(180));

        Double distance = ld.getDistanceIfValid();
        tshi.AutoTurret(257);
        tshi.AutoShooter("start", 1490.14642); // if 14 put 59

        Actions.runBlocking(
                drive.actionBuilder(pos3)
                        .strafeToLinearHeading(new Vector2d(-14, -57), Math.toRadians(160))
                        .build());

        // SHOOTING SEQUENCE //
        sleep(1000);
        tshi.AutoIntakeandShoot("shoot", 1);
        sleep(2500);
        tshi.AutoShooter("stop",0);
        // // // // // // // //

    }

    private void intakeBalls2(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos = new Pose2d(-14, -57, Math.toRadians(160));

        tshi.AutoIntakeandShoot("intake", 1);

        Actions.runBlocking(
                drive.actionBuilder(pos)
                        .strafeToLinearHeading(new Vector2d(-20, -23), Math.toRadians(180))
                        .build());

        sleep(500);

        Pose2d pos2 = new Pose2d(-20, -23, Math.toRadians(180));

        Actions.runBlocking(
                drive.actionBuilder(pos2)
                        .strafeToLinearHeading(new Vector2d(-64, -23), Math.toRadians(190),  new TranslationalVelConstraint(70), new ProfileAccelConstraint(-90, 90))
                        .build());

        tshi.AutoIntakeandShoot("stop", 0);

        /*Pose2d pos3 = new Pose2d(-60, -20, Math.toRadians(180));

        Actions.runBlocking(
                drive.actionBuilder(pos3)
                        .strafeTo(new Vector2d(-14, -20), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-90,90))
                        .strafeTo(new Vector2d(-14, 0),new TranslationalVelConstraint(30), new ProfileAccelConstraint(-20, 90))
                        .build());


        sleep(300);*/

    }

    private void shootAgain(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos = new Pose2d(-64, -23, Math.toRadians(190));

        Double distance = ld.getDistanceIfValid();
        tshi.AutoLS(0.5);
        tshi.AutoTurret(410);
        tshi.AutoShooter("start", 1264.64553); // if 14 put 59

        Actions.runBlocking(
                drive.actionBuilder(pos)
                        .strafeToLinearHeading(new Vector2d(-14, 0), Math.toRadians(200))
                        .build());

        // SHOOTING SEQUENCE //
        sleep(1000);
        tshi.AutoIntakeandShoot("shoot", 1);
        sleep(2500);
        tshi.AutoShooter("stop",0);
        // // // // // // // //

        Pose2d pos2 = new Pose2d(-14, 0, Math.toRadians(200));

        Actions.runBlocking(
                drive.actionBuilder(pos2)
                        .strafeToLinearHeading(new Vector2d(-40,-5 ), Math.toRadians(180), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-20, 90))
                        .build());

        tshi.AutoTurret(0);
        sleep(300);


    }
}