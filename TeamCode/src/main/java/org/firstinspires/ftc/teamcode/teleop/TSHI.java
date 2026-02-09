package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.auto.Trig;
import org.firstinspires.ftc.teamcode.prism.GoBildaPrismDriver;


public class TSHI {
    DcMotor turretMotor, intake;
    DcMotorEx Lshooter, Rshooter;
    Servo lsservo,light ;
    CRServo lIntake, rIntake;
    GoBildaPrismDriver prism;

    // Turret PID constants
    private double turretKP = 0.005;
    private double turretKI = 0.0;
    private double turretKD = 0.0003;

    private double turretIntegral = 0;
    private double turretLastError = 0;

    private int maxTurretTicks = 754;
    private int minTurretTicks = -745;

    private double maxVelocity = 1600;

    //Flywheel Tuning Variables
    public double lowVelocity = 0.0;
    public double accVelocity = 0.0;
    public double velocity;

    public double hoodPos = 0.0;

    double currTargetvelocity = accVelocity;

    double F = 14.620;
    double P = 170;

    double[] stepSizesVel = {100.0, 50.0, 10.0, 1.0};
    double[] stepSizesHood = { 0.1, 0.01 ,0.001, 0.0001};

    int stepIndex = 1;


    public TSHI(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotor.class, "trrt");
        Lshooter = hardwareMap.get(DcMotorEx.class, "LS");
        Rshooter = hardwareMap.get(DcMotorEx.class, "RS");
        intake = hardwareMap.get(DcMotor.class, "intake");
        lsservo = hardwareMap.get(Servo.class, "lshood");
        lIntake = hardwareMap.get(CRServo.class, "leftIntake");
        rIntake = hardwareMap.get(CRServo.class, "rightIntake");
        prism = hardwareMap.get(GoBildaPrismDriver.class,"LED");
        light = hardwareMap.get(Servo.class, "RGB");


        //    distance= new LimelightDistance(hardwareMap);
        lsservo.setPosition(0);


        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Lshooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rshooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rshooter.setDirection(DcMotorEx.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        Lshooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        Rshooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        light.setPosition(1);




    }
    public double getpos(){
        double tpos = turretMotor.getCurrentPosition();
        return tpos;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    public void LSActions(LimelightDistance limelightDistance) {

        if(limelightDistance.getDistanceIfValid()!=null){
            Double distance = limelightDistance.getDistanceIfValid();

            if (distance >= 100.0){
                lsservo.setPosition(0.9);
                velocity = limelightDistance.getHighPowerIfValid(distance);
            }
            if (distance < 50.0){
                lsservo.setPosition(0.1);
                velocity = limelightDistance.getLowLowPowerIfValid(distance);
            }
            if (distance < 100.00 && distance >= 50.0){
                lsservo.setPosition(0.5);
                velocity = limelightDistance.getLowPowerIfValid(distance);
            }
        }

    }
    /*public void LSActionsTrig(Trig trig ) {

        if (trig.distance() >= 100.0){
            lsservo.setPosition(0.9);
            velocity = trig.velocityGreaterThan100();
        }
        if (trig.distance() < 50.0){
            lsservo.setPosition(0.1);
            velocity = trig.velocityLessThan50();
        }
        if (trig.distance() < 100.00 && trig.distance() >= 50.0){
            lsservo.setPosition(0.5);
            velocity = trig.velocityGreaterThan50LessThan100();
        }

    }*/
    public void Intake(Gamepad gamepad) {

        if (gamepad.rightBumperWasPressed()) {
            intake.setPower(-1);
            rIntake.setPower(1);
            lIntake.setPower(-1);
        }
        if (gamepad.leftBumperWasPressed()) {
            intake.setPower(1);
            rIntake.setPower(1);
            lIntake.setPower(-1);
        }
        if (gamepad.rightBumperWasReleased() || gamepad.leftBumperWasReleased()) {
            intake.setPower(0);
            rIntake.setPower(0);
            lIntake.setPower(0);
        }
        if (gamepad.aWasPressed()) {
            lIntake.setPower(1);
            rIntake.setPower(-1);
            intake.setPower(-1);
        }
        if (gamepad.aWasReleased()) {
            lIntake.setPower(0);
            rIntake.setPower(0);
            intake.setPower(0);

        }
    }
    public void ShooterActions(Gamepad gamepad,double color,LimelightDistance limelightDistance) {

        if (gamepad.right_trigger > 0){
            Lshooter.setVelocity(velocity);
            Rshooter.setVelocity(velocity);
        }
        if(gamepad.right_trigger == 0){
            Lshooter.setVelocity(0);
            Rshooter.setVelocity(0);
        }
        if((Lshooter.getVelocity()<velocity+50 && Lshooter.getVelocity()>velocity-50 && Lshooter.getVelocity()!=0)){
            light.setPosition(0.500);
        }
        if (Lshooter.getVelocity() < velocity - 50 || !limelightDistance.getLatestResult().isValid()) {
            light.setPosition(color);
        }




       /*Lshooter.setPower(gamepad.right_trigger);
       Rshooter.setPower(-gamepad.right_trigger);*/
        /*if (gamepad.right_trigger > 0) {
            Lshooter.setPower(speed);
            Rshooter.setPower(-speed);

        }
        if (gamepad.right_trigger == 0){
            Lshooter.setPower(0);
            Rshooter.setPower(0);

        }*/
    }

    public void TurretJoystickControl(Gamepad gamepad) {
        double lx = gamepad.left_stick_x;
        double ly = gamepad.left_stick_y;

        double deadzone = 0.15;

        if (Math.hypot(lx, ly) <= deadzone) {
            turretMotor.setPower(0);
            turretIntegral = 0;
            return;
        }

        double rawDeg = Math.toDegrees(Math.atan2(lx, -ly));
        double targetDeg = (rawDeg < 0) ? rawDeg + 360 : rawDeg;

        final int TICKS_PER_REV = 6090;
        double ticksPerDeg = TICKS_PER_REV / 360.0;

        int targetTicksMod = (int) Math.round(targetDeg * ticksPerDeg);

        int currentTicks = turretMotor.getCurrentPosition();
        int currentMod = currentTicks % TICKS_PER_REV;
        if (currentMod < 0) currentMod += TICKS_PER_REV;

        int delta = targetTicksMod - currentMod;
        if (delta > TICKS_PER_REV / 2) delta -= TICKS_PER_REV;
        else if (delta < -TICKS_PER_REV / 2) delta += TICKS_PER_REV;

        int targetTicks = currentTicks + delta;

        // Clamp to soft limits
        targetTicks = (int) clamp(targetTicks, minTurretTicks, maxTurretTicks);

        // PID control
        int error = targetTicks - currentTicks;

        if (Math.abs(error) < 5) {
            turretMotor.setPower(0);
            turretIntegral = 0;
            return;
        }

        turretIntegral += error;
        double derivative = error - turretLastError;
        turretLastError = error;

        double power = turretKP * error +
                turretKI * turretIntegral +
                turretKD * derivative;

        power = clamp(power, -0.6, 0.6);

        // Safety: prevent pushing past limits
        if ((currentTicks >= maxTurretTicks && power > 0) ||
                (currentTicks <= minTurretTicks && power < 0)) {
            power = 0;
        }

        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setPower(power);
    }


    public void AutoIntakeandTransfer(String pos) {
        pos.toLowerCase();


        if (pos == "intakerun") {
            intake.setPower(-1);
        }
        if (pos == "intakestop") {
            intake.setPower(0);
        }
        if (pos == "intakereverse") {
            intake.setPower(1);
        }
        if (pos == "intakeslow") {
            intake.setPower(.6);
        }

    }
    public void AutoShooter(String status, double velocity) {



        if (status == "start") {
            Lshooter.setVelocity(velocity);
            Rshooter.setVelocity(velocity);
        }
        if ("stop".equalsIgnoreCase(status)) {
            Lshooter.setVelocity(0);
            Rshooter.setVelocity(0);
        }

    }

    public void AutoLS(double pos) {
        lsservo.setPosition(pos);
    }

    public void AutoTurret(int pos) {
        turretMotor.setTargetPosition(pos);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.7);
    }

    public void AutoIntakeandShoot(String stat, double speed) {

        if ("shoot".equalsIgnoreCase(stat)) {
            lIntake.setPower(speed);
            rIntake.setPower(-speed);
            intake.setPower(-speed);
        }
        if ("intake".equalsIgnoreCase(stat)) {
            lIntake.setPower(-speed);
            rIntake.setPower(speed);
            intake.setPower(-speed);
        }
        if ("stop".equalsIgnoreCase(stat)) {
            lIntake.setPower(0);
            rIntake.setPower(0);
            intake.setPower(0);
        }

    }

    public void FlywheelTuning(Gamepad gamepad){
        if (gamepad.yWasPressed()){
            if (currTargetvelocity == accVelocity){
                currTargetvelocity = lowVelocity;
            } else { currTargetvelocity = accVelocity;}
        }
        if(gamepad.xWasPressed()){
            currTargetvelocity = 0;
        }

        if (gamepad.bWasPressed()){
            stepIndex = (stepIndex + 1) % stepSizesVel.length;
        }
        if (gamepad.dpadLeftWasPressed()){
            accVelocity -= stepSizesVel[stepIndex];
        }
        if (gamepad.dpadRightWasPressed()){
            accVelocity += stepSizesVel[stepIndex];
        }
        if (gamepad.dpadDownWasPressed()){
            hoodPos -= stepSizesHood[stepIndex];
        }
        if (gamepad.dpadUpWasPressed()){
            hoodPos += stepSizesHood[stepIndex];
        }


        Lshooter.setVelocity(currTargetvelocity);
        Rshooter.setVelocity(currTargetvelocity);
        lsservo.setPosition(hoodPos);


    }
    public void backup(Gamepad gamepad, double color){
        if (gamepad.xWasPressed()){
            lsservo.setPosition(0.1);
            light.setPosition(0.722);
            turretMotor.setTargetPosition(280);
            turretMotor.setPower(0.5);
            Lshooter.setVelocity(1145);
            Rshooter.setVelocity(1145);
        }
        if (gamepad.bWasPressed()){
            lsservo.setPosition(0.5);
            light.setPosition(0.722);
            turretMotor.setTargetPosition(290);
            turretMotor.setPower(0.5);
            Lshooter.setVelocity(1285);
            Rshooter.setVelocity(1285);
        }
        if (gamepad.yWasPressed()){
            lsservo.setPosition(0.9);
            light.setPosition(0.722);
            turretMotor.setTargetPosition(170);
            turretMotor.setPower(0.5);
            Lshooter.setVelocity(1535);
            Rshooter.setVelocity(1535);
        }
        if (gamepad.xWasReleased() && gamepad.yWasReleased() && gamepad.bWasReleased()){
            lsservo.setPosition(0.1);
            light.setPosition(color);
            turretMotor.setTargetPosition(0);
            turretMotor.setPower(-0.5);
            Lshooter.setVelocity(0);
            Rshooter.setVelocity(0);
        }
    }

    public void LSActionsNotValidMode(Double distance) {

        double m = 3.33499;
        double b = 1101.85831; // 1071.85831

        lsservo.setPosition(0.9);
        velocity = (m*distance)+b;

    }
    public void ShooterActionsNotValidMode(Gamepad gamepad) {

        if (gamepad.left_trigger > 0) {
            Lshooter.setVelocity(velocity);
            Rshooter.setVelocity(velocity);
        }
        if (gamepad.left_trigger == 0) {
            Lshooter.setVelocity(0);
            Rshooter.setVelocity(0);
        }

    }

}
