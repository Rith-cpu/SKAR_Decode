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
public class AutoRedFar extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        TSHI tshi = new TSHI(hardwareMap);
        TurretTesting tt = new TurretTesting(hardwareMap);
        LimelightDistance ld = new LimelightDistance(hardwareMap);

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {

            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(14, -70, Math.toRadians(-90)));

            tshi.AutoIntakeandTransfer("transdown");
            tshi.AutoLS(0.9);
            tshi.AutoTurret(240);

            tt.setPipeline(0);
            tt.getLimelightStatus();

            waitForStart();

            shootPreloads(drive, tshi, tt, ld);

            intakeBalls(drive, tshi, tt, ld);

            shootSecondLoad(drive, tshi, tt, ld);

        }

    }
    private void shootPreloads(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos = new Pose2d(14, -70, Math.toRadians(-90));

        Double distance = ld.getDistanceIfValid();
        //tt.updateLL();   // poll Limelight so telemetry stays live
        // tt.runLL();      // always auto-align the turret

        tshi.AutoShooter("start", ld.estimatePowerFromDistanceGreaterThan100(distance) - 0.02); // if 14 put 59

        Actions.runBlocking(
                drive.actionBuilder(pos)
                        .strafeTo(new Vector2d(14, -60),new TranslationalVelConstraint(100), new ProfileAccelConstraint(-90, 90))
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
        tshi.AutoShooter("start", ld.estimatePowerFromDistanceGreaterThan100(distance) - 0.03); // if 14 put 59
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
        Pose2d pos = new Pose2d(14, -60, Math.toRadians(-90));
        int count = 0;

        tshi.AutoIntakeandTransfer("intakerun");

        while (count == 0) {
            Actions.runBlocking(
                    drive.actionBuilder(pos)
                            .strafeToSplineHeading(new Vector2d(27, -53), 0)
                            .strafeTo(new Vector2d(65, -53), new TranslationalVelConstraint(15), new ProfileAccelConstraint(-10, 90))
                            .build());
        count++;
        }
        if (count == 1) {
            tshi.AutoIntakeandTransfer("intakestop");
        }

        Pose2d pos2 = new Pose2d(65, -53, 0);

        Actions.runBlocking(
                drive.actionBuilder(pos2)
                        .strafeToSplineHeading(new Vector2d(14, -60), Math.toRadians(-90))
                        .build());

    }

    private void shootSecondLoad(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos = new Pose2d(14, -60, Math.toRadians(-90));

        Double distance = ld.getDistanceIfValid();
        tshi.AutoShooter("start", ld.estimatePowerFromDistanceGreaterThan100(distance) + 0.05); // if 14 put 59
        tshi.AutoTurret(235);

        // // // // FIRST BALL // // // //
        sleep(3000);
        tshi.AutoIntakeandTransfer("transup");
        sleep(300);
        // // // // SECOND BALL // // // //
        tshi.AutoIntakeandTransfer("transdown");
        tshi.AutoIntakeandTransfer("intakereverse");
        sleep(15);
        tshi.AutoIntakeandTransfer("intakestop");
        sleep(100);
        tshi.AutoIntakeandTransfer("intakerun");
        sleep(900);
        tshi.AutoIntakeandTransfer("intakestop");
        sleep(700);
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
                        .strafeTo(new Vector2d(17, -48), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-20, 90))
                        .build());
    }
}