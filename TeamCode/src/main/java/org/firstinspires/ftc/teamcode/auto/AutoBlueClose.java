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
public class AutoBlueClose extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        TSHI tshi = new TSHI(hardwareMap);
        TurretTesting tt = new TurretTesting(hardwareMap);
        LimelightDistance ld = new LimelightDistance(hardwareMap);
        Double distance = ld.getDistanceIfValid();

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {

            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-51, 50, Math.toRadians(315)));

            tshi.AutoIntakeandTransfer("transdown");
            tshi.AutoLS(0.1);
            // tshi.AutoTurret(230);

            tt.setPipeline(1);
            tt.getLimelightStatus();

            waitForStart();

            shootPreloads(drive, tshi, tt, ld);

            PickUpBALLS(drive, tshi, tt, ld);

            ShootSecond(drive, tshi, tt, ld);

        }

    }
    private void shootPreloads(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld) {
        Pose2d pos1 = new Pose2d(-51, 50, Math.toRadians(315));

        Double distance = ld.getDistanceIfValid();
        //tt.updateLL();   // poll Limelight so telemetry stays live
        // tt.runLL();      // always auto-align the turret
        int count = 0;


        while (count == 0) {
            tshi.AutoShooter("start",0.47);
            Actions.runBlocking(
                    drive.actionBuilder(pos1)
                            .strafeTo(new Vector2d(-35, 38), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-90, 90))
                            .build());
            count++;
        }

        while (count == 0 || count == 1) {
            sleep(2200);
            tshi.AutoIntakeandTransfer("transup");
            sleep(300);
            tshi.AutoIntakeandTransfer("transdown");
            sleep(100);
            tshi.AutoIntakeandTransfer("intakerun");
            sleep(900);
            tshi.AutoIntakeandTransfer("intakestop");
            sleep(500);
            tshi.AutoIntakeandTransfer("transup");
            sleep(300);
            tshi.AutoIntakeandTransfer("transdown");
            sleep(1000);
            tshi.AutoIntakeandTransfer("intakerun");
            sleep(1700);
            tshi.AutoIntakeandTransfer("transup");
            sleep(600);
            tshi.AutoIntakeandTransfer("transdown");
            tshi.AutoIntakeandTransfer("intakerun");
            sleep(1000);
            count++;
        }


    }
    private void PickUpBALLS(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld){
        Pose2d pos1 = new Pose2d(-35, 38, Math.toRadians(315));
        int count = 2;


        while (count == 2){
            Actions.runBlocking(
                    drive.actionBuilder(pos1)
                            .strafeToLinearHeading(new Vector2d(-18, 25),Math.toRadians(180))
                            .build());
            count++;
        }
        Pose2d pos2 = new Pose2d(new Vector2d(-18, 25),Math.toRadians(180));
        tshi.AutoIntakeandTransfer("intakerun");
        Actions.runBlocking(
                drive.actionBuilder(pos2)
                        .strafeTo(new Vector2d(-49, 28), new TranslationalVelConstraint(100), new ProfileAccelConstraint(-20, 90) )
                        .build());
        tshi.AutoIntakeandTransfer("intakestop");

    }
    private void ShootSecond(MecanumDrive drive, TSHI tshi, TurretTesting tt, LimelightDistance ld){

        Pose2d pos1 = new Pose2d(new Vector2d(-49, 28),Math.toRadians(180));

        int count = 0;


        while (count == 0) {
            Actions.runBlocking(
                    drive.actionBuilder(pos1)
                            .strafeToSplineHeading(new Vector2d(-30, 34), Math.toRadians(315))
                            .build());
            count++;
        }

        while (count == 1) {
            sleep(2200);
            tshi.AutoIntakeandTransfer("transup");
            sleep(300);
            tshi.AutoIntakeandTransfer("transdown");
            sleep(100);
            tshi.AutoIntakeandTransfer("intakerun");
            sleep(900);
            tshi.AutoIntakeandTransfer("intakestop");
            sleep(500);
            tshi.AutoIntakeandTransfer("transup");
            sleep(300);
            tshi.AutoIntakeandTransfer("transdown");
            sleep(1000);
            tshi.AutoIntakeandTransfer("intakerun");
            sleep(1700);
            tshi.AutoIntakeandTransfer("intakestop");
            sleep(600);
            tshi.AutoIntakeandTransfer("transup");
            sleep(600);
            tshi.AutoIntakeandTransfer("transdown");
            tshi.AutoTurret(0);
            sleep(1000);
            count++;
        }
        Pose2d pos2 = new Pose2d(new Vector2d(-35, 38),Math.toRadians(315));

        Actions.runBlocking(
                drive.actionBuilder(pos2)
                        .strafeTo(new Vector2d(-48, 30))
                        .build());
    }



}


