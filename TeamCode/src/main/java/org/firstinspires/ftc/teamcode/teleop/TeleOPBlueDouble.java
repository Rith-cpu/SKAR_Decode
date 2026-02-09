package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.prism.GoBildaPrismDriver;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOP - Blue - 2P")
public class TeleOPBlueDouble extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        TSHI tshi = new TSHI(hardwareMap);
        Drivetrain driveTrainControl = new Drivetrain(hardwareMap);
        TurretTesting tt = new TurretTesting(hardwareMap);
        LimelightDistance distance = new LimelightDistance(hardwareMap);
        tshi.prism.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_1);

        tt.setPipeline(1);
        tt.getLimelightStatus();
        // Optional: tune these later; start with defaults in TurretTesting
        tt.setGains(0.03, 1.0, 1.0);
        //SensorSparkFunOTOS sparkFunControl = new SensorSparkFunOTOS(hardwareMap);
        //sparkFunControl.configureOtos(telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {

            tt.updateLL();   // poll Limelight so telemetry stays live
            tt.runLL();      // always auto-align the turret

            // If auto-aim is not running this loop, the last Limelight values may be stale.
            // This keeps telemetry truthful.
            telemetry.addData("LL status", tt.getLimelightStatus());
            telemetry.addData("LL valid", tt.hasValidTarget());
            telemetry.addData("LL tx", tt.getTx());



            Double llDistance = distance.getDistanceIfValid();

            if (llDistance != null) {
                tshi.LSActions(distance);
                telemetry.addData("LL Distance", llDistance);
                if (llDistance < 100){
                    telemetry.addData("Low speed", distance.getLowPowerIfValid(llDistance));
                }
                else{
                    telemetry.addData("High Speed", distance.getHighPowerIfValid(llDistance));
                }

                //  turretShooterHoodIntake.

            } else {
                telemetry.addData("LL Distance", "N/A");
            }



            telemetry.addData("Shooter Speed (Motor)", tshi.Lshooter.getPower());
            telemetry.addData("LS Position", tshi.lsservo.getPosition());
            telemetry.addData("Turret Position", tshi.turretMotor.getCurrentPosition());
            telemetry.update();

            driveTrainControl.Driving(gamepad1);
            tshi.ShooterActions(gamepad2,0.611,distance);
            tshi.Intake(gamepad2);


            //Tempprary manual control
            //turretShooterHoodIntake.TurretJoystickControl(gamepad2);
            //tt.runLL();


            //sparkFunControl.OTOS(gamepad1,telemetry);

        }
    }
}
