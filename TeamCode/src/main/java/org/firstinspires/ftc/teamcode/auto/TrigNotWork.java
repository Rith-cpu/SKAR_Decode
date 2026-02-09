package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import com.acmerobotics.roadrunner.Pose2d;


public class TrigNotWork {
    DcMotor turretMotor;
    private MecanumDrive drive;


    private int maxTurretTicks = 754;
    private int minTurretTicks = -745;
    // Turret PID constants
    private double kP = 0.005;
    private double kI = 0.0;
    private double kD = 0.0003;

    private double integral = 0;
    private double lastError = 0;
    // Goal + current robot position (in whatever units your localizer uses)
    private double goalX = 50;
    private double goalY = 50;

    // These MUST be updated continuously from your OpMode loop
    private double currPosX;
    private double currPosY;


    public TrigNotWork(HardwareMap init, MecanumDrive drive){
        this.drive = drive;
        turretMotor = init.get(DcMotor.class, "trrt");

        Pose2d pose = drive.localizer.getPose();
        this.currPosX = pose.position.x;
        this.currPosY = pose.position.y;
    }

    /** Call this every loop to keep Trig using your live robot pose. */
    public void setCurrentPose(double x, double y) {
        this.currPosX = x;
        this.currPosY = y;
    }

    /** Optional: update your target on the fly (ex: switching scoring positions). */
    public void setGoal(double x, double y) {
        this.goalX = x;
        this.goalY = y;
    }

    public double getCurrPosX() { return currPosX; }
    public double getCurrPosY() { return currPosY; }
    public double getGoalX() { return goalX; }
    public double getGoalY() { return goalY; }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }

    /** Pull live pose from RoadRunner MecanumDrive localizer */
    public void refreshPoseFromDrive() {
        Pose2d pose = drive.localizer.getPose();
        currPosX = pose.position.x;
        currPosY = pose.position.y;
    }

    //These funcs for getting speed based off of distance, which is given by the current robot pos and goal pos
    public double distance(){
        // Ensure we compute distance using the latest pose (even if called before autoAlignTurret in the loop)
        refreshPoseFromDrive();
        return Math.hypot(goalX - currPosX, goalY - currPosY);
    }
    public double velocityGreaterThan100(){
        double m = 0;
        double b = 0;
        return m*this.distance()+b;
    }
    public double velocityGreaterThan50LessThan100(){
        double m = 0;
        double b = 0;
        return m*this.distance()+b;
    }
    public double velocityLessThan50(){
        double m = 0;
        double b = 0;
        return m*this.distance()+b;
    }
    //These funcs for turning the turret based off of calculated angle
    public double turretAngle() {
        refreshPoseFromDrive();
        return Math.atan2(goalY - currPosY, goalX - currPosX);
    }
    // Auto-align turret to the goal position (robot does not move)
    public void autoAlignTurret() {

        // Pull live RoadRunner pose every loop
        refreshPoseFromDrive();

        // Angle from robot to goal (radians)
        double angleRad = turretAngle();
        double angleDeg = Math.toDegrees(angleRad);
        if (angleDeg < 0) angleDeg += 360;

        final int TICKS_PER_REV = 6090;
        double ticksPerDeg = TICKS_PER_REV / 360.0;

        int targetTicksMod = (int) Math.round(angleDeg * ticksPerDeg);

        int currentTicks = turretMotor.getCurrentPosition();
        int currentMod = currentTicks % TICKS_PER_REV;
        if (currentMod < 0) currentMod += TICKS_PER_REV;

        int delta = targetTicksMod - currentMod;

        // Shortest-path wrap
        if (delta > TICKS_PER_REV / 2) delta -= TICKS_PER_REV;
        else if (delta < -TICKS_PER_REV / 2) delta += TICKS_PER_REV;

        int targetTicks = currentTicks + delta;

        // Apply soft limits
        targetTicks = (int) clamp(targetTicks, minTurretTicks, maxTurretTicks);

        int error = targetTicks - currentTicks;

        // Deadband to prevent jitter
        if (Math.abs(error) < 5) {
            turretMotor.setPower(0);
            integral = 0;
            return;
        }

        // PID math
        integral += error;
        double derivative = error - lastError;
        lastError = error;

        double power = kP * error +
                       kI * integral +
                       kD * derivative;

        power = clamp(power, -0.6, 0.6);

        // Safety: prevent pushing past soft limits
        if ((currentTicks >= maxTurretTicks && power > 0) ||
            (currentTicks <= minTurretTicks && power < 0)) {
            power = 0;
        }

        // Ensure we're in a mode that lets us drive power while reading encoder position
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setPower(power);
    }



}
