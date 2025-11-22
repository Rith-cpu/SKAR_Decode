
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOP")
public class TeleOP extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        TSHI turretShooterHoodIntake = new TSHI(hardwareMap);
        Drivetrain driveTrainControl = new Drivetrain(hardwareMap);
        Limelight ll = new Limelight(hardwareMap);
        //SensorSparkFunOTOS sparkFunControl = new SensorSparkFunOTOS(hardwareMap);
        //sparkFunControl.configureOtos(telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {


            turretShooterHoodIntake.Lshooter.setPower(-0.4);
            turretShooterHoodIntake.Rshooter.setPower(0.4);
            telemetry.addData("Shooter Speed", -gamepad2.right_stick_y + "%");
            telemetry.addData("LS Position", turretShooterHoodIntake.lsservo.getPosition());
            telemetry.addData("Turret Position", turretShooterHoodIntake.turretMotor.getCurrentPosition());
            telemetry.update();
            driveTrainControl.Driving(gamepad1);
            turretShooterHoodIntake.LSActions(gamepad2);
            turretShooterHoodIntake.ShooterActions(gamepad2);
            turretShooterHoodIntake.IntakeandTransfer(gamepad2);
            turretShooterHoodIntake.TurretJoystickControl(gamepad2);
            ll.setTargetId(24);
            ll.runLL();
            ll.setGains(0.015, 1.0,1.0);


            //sparkFunControl.OTOS(gamepad1,telemetry);

        }
    }
}
