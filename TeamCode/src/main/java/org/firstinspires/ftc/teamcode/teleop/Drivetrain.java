package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Drivetrain {

    DcMotor LF_motor, RF_motor;
    DcMotor LB_motor, RB_motor;



    public Drivetrain(HardwareMap initMotors) {
        LF_motor = initMotors.get(DcMotor.class, "LF");
        RF_motor = initMotors.get(DcMotor.class, "RF");
        LB_motor = initMotors.get(DcMotor.class, "LB");
        RB_motor = initMotors.get(DcMotor.class, "RB");
        RF_motor.setDirection(DcMotorSimple.Direction.REVERSE);
       //  RB_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        LF_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        LB_motor.setDirection(DcMotorSimple.Direction.REVERSE);




    }


    public void Driving(Gamepad gamepad) {


        float LeftStickX = gamepad.left_stick_x;
        float RightStickX = gamepad.right_stick_x;
        float RightStickY = gamepad.right_stick_y;
        float LeftStickY = gamepad.left_stick_y;

        double LF_power = 0;
        double LB_power = 0;
        double RF_power = 0;
        double RB_power = 0;

        // Moving forwards & backwards
        if (LeftStickY != 0) {
            LF_power += LeftStickY;
            LB_power += LeftStickY;
            RF_power -= LeftStickY;
            RB_power -= LeftStickY;
        }

        // Strafing Left and Right
        if (LeftStickX != 0) {

            LF_power -= LeftStickX;
            LB_power += LeftStickX;
            RF_power -= LeftStickX;
            RB_power += LeftStickX;

        }

        //Turning Left and Rightcc
        if (RightStickX != 0) {

            LF_power -= RightStickX;
            LB_power -= RightStickX;
            RF_power -= RightStickX;
            RB_power -= RightStickX;

        }

    /* This is where you use the in built math library to set a range between -1 to 1
    First it looks at whether the value is greater than 1 or not by using the Math.min function, and if so, it will set the value to 1
    Next it looks at whether the value is less than -1 or not by using the Math.max function, and if so, it will set the value to -1
    */

        LF_power = Math.max(-1, Math.min(1, LF_power));
        LB_power = Math.max(-1, Math.min(1, LB_power));
        RF_power = Math.max(-1, Math.min(1, RF_power));
        RB_power = Math.max(-1, Math.min(1, RB_power));


        // Now you're setting the values of the power to the motors
        LF_motor.setPower(-LF_power);
        LB_motor.setPower(-LB_power);
        RF_motor.setPower(RF_power);
        RB_motor.setPower(RB_power);

        // Ensure speed is within valid range (-1 to 1)
        LF_power = Math.max(-1, Math.min(1, LF_power));
        LB_power = Math.max(-1, Math.min(1, LB_power));
        RF_power = Math.max(-1, Math.min(1, RF_power));
        RB_power = Math.max(-1, Math.min(1, RB_power));

        // Set motor power to move backward

    }


    public void moveBackward(double speed){

        LF_motor.setPower(speed); // Left motors move backward
        LB_motor.setPower(speed); // Left motors move backward
        RF_motor.setPower(speed);  // Right motors move backward (opposite direction)
        RB_motor.setPower(speed);  // Right motors move backward (opposite direction)
    }

    /* This approach is much more efficient considered to the first joysitck drivetrain for the reason that you don't need to create multiple threads,
    and that you don't need to have so many if statments checking for specific conditions of the controller and the joystick.

    This works by creating a variable that stores the postition of the joystick and either adds or subtracts based off of the needed movement

    At the end using the Math max and min functions the power is capped to a range between -1 and 1, as those are the motor's max and min values

    This drivetrain also may feature a tank drive system cause why not, we would just need to add the extras
    */


}
