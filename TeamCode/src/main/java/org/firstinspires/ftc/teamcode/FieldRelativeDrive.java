package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class FieldRelativeDrive {
    private DcMotor LF_motor, LB_motor, RF_motor, RB_motor;
    private IMU imu;

    public void FieldRelativeInit(HardwareMap hwMap) {
        LF_motor = hwMap.get(DcMotor.class, "LF");
        LB_motor = hwMap.get(DcMotor.class, "LB");
        RF_motor = hwMap.get(DcMotor.class, "RF");
        RB_motor = hwMap.get(DcMotor.class, "RB");

        imu = hwMap.get(IMU.class, "imu");

        LF_motor.setDirection(DcMotor.Direction.REVERSE);
        LB_motor.setDirection(DcMotor.Direction.REVERSE);

        LF_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LB_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RB_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RevHubOrientationOnRobot HubDirection = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);

        imu.initialize(new IMU.Parameters(HubDirection));
    }

    public void drive(double forward, double strafe, double rotate) {
        double LF_power = forward + strafe + rotate; // equations for functionality of mecanum drive
        double LB_power = forward - strafe + rotate;
        double RF_power = forward - strafe - rotate;
        double RB_power = forward + strafe - rotate;

        double max_Power = 1.0;
        double max_Speed = 1.0; // not needed at comps

        max_Power = Math.max(max_Power, Math.abs(LF_power)); // checking for absolute max power
        max_Power = Math.max(max_Power, Math.abs(LB_power));
        max_Power = Math.max(max_Power, Math.abs(RF_power));
        max_Power = Math.max(max_Power, Math.abs(RB_power));

        LF_motor.setPower(max_Speed * (LF_power / max_Power)); // sends out robot commands to each motor
        LB_motor.setPower(max_Speed * (LB_power / max_Power));
        RF_motor.setPower(max_Speed * (RF_power / max_Power));
        RB_motor.setPower(max_Speed * (RB_power / max_Power));
    }

    public void driveFieldRelative(double forward, double strafe, double rotate) {
        double theta = Math.atan2(forward, strafe); // converting coords into polar coords
        double r = Math.hypot(strafe, forward);

        theta = AngleUnit.normalizeRadians(theta - // gets current angle and rotates theta based on where the robot is facing
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        double newForward = r * Math.sin(theta);
        double newStrafe = r * Math.cos(theta);

        this.drive(newForward, newStrafe, rotate);




    }
}
