package org.firstinspires.ftc.teamcode.drive.Templates;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.util.RobotHardware;

@Disabled
@Autonomous(name="ThreadBasedAuton", group="Linear Opmode")
public class ThreadBasedAuton extends LinearOpMode
{
    /* Declare OpMode members. */
    RobotHardware robot   = new RobotHardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     tickInch = 34.22 ;

    private static T265Camera slamra = null;

    String CameraPos = "left";
    String left = "left";
    String right = "right";
    String back = "back";
    String front = "front";

    double Y = 0;
    double X = 0;
    double Heading = 0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.rf.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rb.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lf.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lb.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.intake.setDirection(DcMotorSimple.Direction.FORWARD);

        Thread  t265Thread = new T265Thread();

        //Initialize SLAM and T265 Camera 
        initCameraPos();

        initEncoders();

        slamra.start();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.lf.getCurrentPosition(),
                robot.rf.getCurrentPosition(),
                robot.lb.getCurrentPosition(),
                robot.rb.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        t265Thread.start();

        encoderDrive(0.2,  100,  100, 100, 100,30.0);  // S1: Forward 47 Inches with 5 Sec timeout
        sleep(1500);

        t265Thread.interrupt();

        slamra.stop();
    }

    private class T265Thread extends Thread{
        public T265Thread()
        {
            this.setName("T265Thread");
        }

        @Override
        public void run()
        {

            while (!isInterrupted())
            {
                T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();

                if (up == null) return;



                if(CameraPos.equals(left)){
                    // Initalize vars for X and Y coordinates and divide by 0.0254 to convert meters to inches
                    Y = (-1)*(up.pose.getTranslation().getY() / 0.0254);
                    X = (-1)*(up.pose.getTranslation().getX() / 0.0254);
                    Heading = (up.pose.getHeading()) * (-57.295);
                }

                if(CameraPos.equals(right)){
                    // Initalize vars for X and Y coordinates and divide by 0.0254 to convert meters to inches
                    Y = (up.pose.getTranslation().getY() / 0.0254);
                    X = (up.pose.getTranslation().getX() / 0.0254);
                }

                if(CameraPos.equals(back)){
                    // Initalize vars for X and Y coordinates and divide by 0.0254 to convert meters to inches
                    Y = (-1)*(up.pose.getTranslation().getX() / 0.0254);
                    X = (up.pose.getTranslation().getY() / 0.0254);
                }

                if(CameraPos.equals(front)){
                    // Initalize vars for X and Y coordinates and divide by 0.0254 to convert meters to inches
                    Y = (up.pose.getTranslation().getX() / 0.0254);
                    X = (-1)*(up.pose.getTranslation().getY() / 0.0254);
                }


                Translation2d translation = new Translation2d(X, Y);
                Rotation2d rotation = up.pose.getRotation();
            }
        }
    }

    public void encoderDrive(double speed, double lfInches, double rfInches, double lbInches, double rbInches, double timeoutS) {
        int newlFTarget;
        int newrFTarget;
        int newlBTarget;
        int newrBTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newlFTarget = robot.lf.getCurrentPosition() + (int)(lfInches * tickInch);
            newrFTarget = robot.rf.getCurrentPosition() + (int)(rfInches * tickInch);
            newlBTarget = robot.lb.getCurrentPosition() + (int)(lbInches * tickInch);
            newrBTarget = robot.rb.getCurrentPosition() + (int)(rbInches * tickInch);
            robot.lf.setTargetPosition(newlFTarget);
            robot.rf.setTargetPosition(newrFTarget);
            robot.lb.setTargetPosition(newlBTarget);
            robot.rb.setTargetPosition(newrBTarget);

            // Turn On RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.lf.setPower(Math.abs(speed));
            robot.rf.setPower(Math.abs(speed));
            robot.lb.setPower(Math.abs(speed));
            robot.rb.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.lf.isBusy() && robot.rf.isBusy() && robot.lb.isBusy() && robot.rb.isBusy())) {

                if(lfInches < 0){
                    if(Heading > 1){
                        robot.rf.setPower(Math.abs(speed) + 0.5);
                        robot.rb.setPower(Math.abs(speed) + 0.5);
                    }
                    if(Heading < -1){
                        robot.lf.setPower(Math.abs(speed) + 0.5);
                        robot.lb.setPower(Math.abs(speed) + 0.5);
                    }
                    else{
                        robot.lf.setPower(Math.abs(speed));
                        robot.rf.setPower(Math.abs(speed));
                        robot.lb.setPower(Math.abs(speed));
                        robot.rb.setPower(Math.abs(speed));
                    }
                }

                telemetry.addData("Heading", Heading);
                telemetry.addData("Path1",  "Running to %7d :%7d", newlFTarget, newrFTarget, newlBTarget, newrBTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.lf.getCurrentPosition(),
                        robot.rf.getCurrentPosition(),
                        robot.lb.getCurrentPosition(),
                        robot.rb.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.lf.setPower(0);
            robot.rf.setPower(0);
            robot.lb.setPower(0);
            robot.rb.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //  sleep(250);   // optional pause after each move
        }
    }

    public void initEncoders(){
        robot.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void initCameraPos(){
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
        }

        if(CameraPos.equals(left)){
            //Center the camera to its position on the robot
            slamra.setPose(new Pose2d(-31 * 0.0254, -12 * 0.0254, Rotation2d.fromDegrees(0)));
        }

        if(CameraPos.equals(right)){
            //Center the camera to its position on the robot
            slamra.setPose(new Pose2d(31 * 0.0254, 12 * 0.0254, Rotation2d.fromDegrees(0)));
        }

        if(CameraPos.equals(back)){
            //Center the camera to its position on the robot
            slamra.setPose(new Pose2d(-12 * 0.0254, 31 * 0.0254, Rotation2d.fromDegrees(0)));
        }

        if(CameraPos.equals(front)){
            //Center the camera to its position on the robot
            slamra.setPose(new Pose2d(12 * 0.0254, -31 * 0.0254, Rotation2d.fromDegrees(0)));
        }
    }

}
