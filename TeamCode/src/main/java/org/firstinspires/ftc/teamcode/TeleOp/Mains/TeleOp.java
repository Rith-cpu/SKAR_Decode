
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOP")
public class TeleOP extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        TSHI turretShooterHoodIntake = new TSHI(hardwareMap);
        Drivetrain driveTrainControl = new Drivetrain(hardwareMap);
        //SensorSparkFunOTOS sparkFunControl = new SensorSparkFunOTOS(hardwareMap);
        //sparkFunControl.configureOtos(telemetry);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Shooter Speed", -gamepad2.right_stick_y + "%");
            telemetry.addData("Servo Position", turretShooterHoodIntake.lsservo.getPosition());
            telemetry.addData("Turret Position", turretShooterHoodIntake.turretMotor.getCurrentPosition());

            driveTrainControl.Driving(gamepad1);
            turretShooterHoodIntake.LSandIntakeServoActions(gamepad2);
            turretShooterHoodIntake.TurretandShooterActions(gamepad2);
            //sparkFunControl.OTOS(gamepad1,telemetry);

        }
    }
}
