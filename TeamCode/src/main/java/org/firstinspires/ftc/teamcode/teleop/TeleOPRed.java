package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.tuning.PinpointLocalizer;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOP -  Red" )
public class TeleOPRed extends LinearOpMode {

    public final PinpointLocalizer pinpointLocalizer;

    public TeleOPRed(PinpointLocalizer pinpointLocalizer) {
        this.pinpointLocalizer = pinpointLocalizer;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        TSHI tshi = new TSHI(hardwareMap);
        Drivetrain driveTrainControl = new Drivetrain(hardwareMap);
        TurretTesting tt = new TurretTesting(hardwareMap);
        LimelightDistance distance = new LimelightDistance(hardwareMap);

        tt.setPipeline(0);
        tt.getLimelightStatus();
        // Optional: tune these later; start with defaults in TurretTesting
        // tt.setGains(0.04, 1.0, 0.7);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {

            tt.updateLL();   // poll Limelight so telemetry stays live
            tt.runLL();      // always auto-align the turret
            Pose2d startPos = pinpointLocalizer.setPose(new Pose2d(17, 40, Math.toRadians(-90)));
            Pose2d currPose = pinpointLocalizer.getPose();

            // If auto-aim is not running this loop, the last Limelight values may be stale.
            // This keeps telemetry truthful.
            telemetry.addData("LL status", tt.getLimelightStatus());
            telemetry.addData("LL valid", tt.hasValidTarget());
            telemetry.addData("x", currPose.position.x);
            telemetry.addData("y", currPose.position.y );
            telemetry.addData("heading", currPose.heading.toDouble());


            Double llDistance = distance.getDistanceIfValid();



            if (llDistance != null) {
                tshi.LSActions(gamepad1,distance);
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

                //  turretShooterHoodIntake.

            } else {
                telemetry.addData("LL Distance", "N/A");
            }

            if (tshi.transfer.getPosition() == 0.2) {
                telemetry.addData("Transfer Status:", "UP");
            } else if (tshi.transfer.getPosition() == 0.5) {
                telemetry.addData("Transfer Status:", "DOWN");
            }

            telemetry.addData("Shooter Speed (Motor)", tshi.Lshooter.getPower());
            //telemetry.addData("LS Position", turretShooterHoodIntake.lsservo.getPosition());
            telemetry.addData("Turret Position", tshi.turretMotor.getCurrentPosition());
            telemetry.update();

            driveTrainControl.Driving(gamepad1);
            tshi.ShooterActions(gamepad1);
            tshi.IntakeandTransfer(gamepad1);


            //Tempprary manual control
            //turretShooterHoodIntake.TurretJoystickControl(gamepad2);
            //tt.runLL();



        }
    }
}
