package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.PinpointLocalizer;
import com.acmerobotics.roadrunner.Pose2d;


public class Trig {
    DcMotor turretMotor;

    // Optional: if provided, Trig can pull live pose internally each loop
    private PinpointLocalizer pinpointLocalizer;


    private int maxTurretTicks = 754;
    private int minTurretTicks = -745;
    // Turret PID constants
    private double kP = 0.005;
    private double kI = 0.0;
    private double kD = 0.0003;

    private double integral = 0;
    private double lastError = 0;
    // Goal + current robot position (in whatever units your localizer uses)
    private double goalX = 60;
    private double goalY = 60;

    // These MUST be updated continuously from your OpMode loop
    private double currPosX;
    private double currPosY;


    public Trig(HardwareMap init, double x, double y){
        currPosX = x;
        currPosY = y;
        turretMotor = init.get(DcMotor.class, "trrt");
        MecanumDrive drive = new MecanumDrive(init, new Pose2d(0, 0, 0));
        Pose2d pose = drive.localizer.getPose();
    }

    /** Prefer this constructor if you have a PinpointLocalizer; Trig will update pose internally. */
    public Trig(HardwareMap init, PinpointLocalizer localizer){
        turretMotor = init.get(DcMotor.class, "trrt");
        this.pinpointLocalizer = localizer;

        // Seed with the current pose so we don't start at (0,0)
        Pose2d pose = localizer.getPose();
        this.currPosX = pose.position.x;
        this.currPosY = pose.position.y;
    }

    /** Attach a PinpointLocalizer so Trig can fetch live pose internally each loop. */
    public void setLocalizer(PinpointLocalizer localizer) {
        if (localizer != null) {
            Pose2d pose = localizer.getPose();
            this.currPosX = pose.position.x;
            this.currPosY = pose.position.y;
        }
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

    /** Refreshes currPosX/currPosY from Pinpoint if available. Safe to call every loop. */
    private void refreshPoseFromLocalizer() {
        if (pinpointLocalizer != null) {
            pinpointLocalizer.update();
            Pose2d pose = pinpointLocalizer.getPose();
            currPosX = pose.position.x;
            currPosY = pose.position.y;
        }
    }

    //These funcs for getting speed based off of distance, which is given by the current robot pos and goal pos
    public double distance(){
        // Ensure we compute distance using the latest pose (even if called before autoAlignTurret in the loop)
        refreshPoseFromLocalizer();
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
        refreshPoseFromLocalizer();
        return Math.atan2((goalY - currPosY), (goalX - currPosX));
    }
    // Auto-align turret to the goal position (robot does not move)
    public void autoAlignTurret() {

        // Pull live pose every loop (if localizer is attached)
        refreshPoseFromLocalizer();

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
