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

@Autonomous(group = "Blue")
public class AutoBlueFar extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        TSHI tshi = new TSHI(hardwareMap);
        TurretTesting tt = new TurretTesting(hardwareMap);
        LimelightDistance ld = new LimelightDistance(hardwareMap);
        Double distance = ld.getDistanceIfValid();

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {

            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-14, -70, Math.toRadians(-90)));

            tshi.AutoIntakeandTransfer("transdown");
            tshi.AutoLS(0.9);
            tshi.AutoTurret(-190);

            tt.setPipeline(1); // change to 1
            tt.getLimelightStatus();

            waitForStart();

            shootPreloads(drive, tshi, tt, ld);

            intakeBalls(drive, tshi, tt, ld);

            shootSecondLoad(drive, tshi, tt, ld);

        }

    }
    private void shootPreloads(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos = new Pose2d(-14, -70, Math.toRadians(-90));

        Double distance = ld.getDistanceIfValid();
        //tt.updateLL();   // poll Limelight so telemetry stays live
        // tt.runLL();      // always auto-align the turret

        tshi.AutoShooter("start", ld.estimatePowerFromDistanceGreaterThan100(distance) - 0.01);

        Actions.runBlocking(
                drive.actionBuilder(pos)
                        .strafeTo(new Vector2d(-14, -60),new TranslationalVelConstraint(100), new ProfileAccelConstraint(-90, 90))
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
        sleep(2400);
        tshi.AutoIntakeandTransfer("transup");
        sleep(600);
        // // // // STOPS FUNCTIONS // // // //
        tshi.AutoIntakeandTransfer("transdown");
        tshi.AutoTurret(0);
        tshi.AutoIntakeandTransfer("intakestop");
        tshi.AutoShooter("stop", 0);

    }

    private void intakeBalls(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos = new Pose2d(-14, -60, Math.toRadians(-90));

        tshi.AutoIntakeandTransfer("intakerun");

        Actions.runBlocking(
                drive.actionBuilder(pos)
                        .strafeToSplineHeading(new Vector2d(-25, -35),Math.toRadians(180))
                        .strafeTo(new Vector2d(-44, -35), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-20, 90))
                        .build());

        tshi.AutoIntakeandTransfer("intakestop");

        Pose2d pos2 = new Pose2d(-44, -35, 180);

        Actions.runBlocking(
                drive.actionBuilder(pos2)
                        .strafeToSplineHeading(new Vector2d(-14, -60), Math.toRadians(-90))
                        .build());

    }

    private void shootSecondLoad(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos = new Pose2d(-14, -60, Math.toRadians(-90));
        Double distance = ld.getDistanceIfValid();

        tshi.AutoShooter("start", ld.estimatePowerFromDistanceGreaterThan100(distance) + 0.05);

        tshi.AutoTurret(-180);

        // // // // FIRST BALL // // // //
        sleep(2500);
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
        sleep(2400);
        tshi.AutoIntakeandTransfer("transup");
        sleep(600);
        // // // // STOPS FUNCTIONS // // // //
        tshi.AutoIntakeandTransfer("transdown");
        tshi.AutoTurret(0);
        tshi.AutoIntakeandTransfer("intakestop");
        tshi.AutoShooter("stop", 0);

        Actions.runBlocking(
                drive.actionBuilder(pos)
                        .strafeTo(new Vector2d(-14, -48), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-20, 90))
                        .build());
    }
}