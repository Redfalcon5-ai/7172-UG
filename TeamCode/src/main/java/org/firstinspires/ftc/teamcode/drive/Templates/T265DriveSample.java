package org.firstinspires.ftc.teamcode.drive.Templates;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.spartronics4915.lib.T265Camera;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

@Disabled
@TeleOp(name="T265DriveSample", group="Linear Opmode")
public class T265DriveSample extends LinearOpMode
{
    //Create Robot Hardware Object
    RobotHardware robot = new RobotHardware();

    //Create Variables for Motor/Servo Powers
    double linearPos = 0.5;
    double intakePow = 0.0;
    double conveyorPow = 0.0;
    double flyPow = 0.0;

    //Create T265 Camera Object
    private static T265Camera slamra = null;

    //Set Camera's Position on the robot (Mount with wire on the left)
    String CameraPos = "left";
    String left = "left";
    String right = "right";
    String back = "back";
    String front = "front";

    //Instance Variables for threading
    double Y = 0;
    double X = 0;
    double Heading = 0;

    @Override
    public  void runOpMode() {
        //Initialize Hardware and reverse motors
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);
        robot.rf.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rb.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lf.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lb.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.intake.setDirection(DcMotorSimple.Direction.FORWARD);

        //Create T265 Thread
        Thread t265Thread = new T265Thread();

        //Init SLAM and the camera with the correct starting coordinates
        initCameraPos();

        waitForStart();

        slamra.start();

        //Start thread and continue with the main thread
        t265Thread.start();

        while (opModeIsActive()) {
            //Driving Controls
            double leftStickY = -gamepad1.left_stick_y; //Forward and Backward
            double rightStickY = -gamepad1.right_stick_y;   //Forward and Backward
            double leftStickX = gamepad1.left_stick_x;  //Turning
            double rightStickX = gamepad1.right_stick_x;    //Strafing
            mecDrive(leftStickY, rightStickY, leftStickX, rightStickX);

            //Latch Controls
            if (conveyorPow >= -1 && conveyorPow <= 1) {
                conveyorPow = 0;
            }
            if (flyPow >= -1 && flyPow <= 1) {
                flyPow = 0;
            }
            if (intakePow >= -1 && intakePow <= 1) {
                intakePow = 0;
            }

            //Distance Sensor controls
            if (((DistanceSensor) robot.colorv3).getDistance(DistanceUnit.CM) < 4) {
                flyPow = 2;
                conveyorPow = 2;
            }

            //Trigger Controls
            if (gamepad1.right_trigger > 0) {
                intakePow = 1;
                conveyorPow = 1;
            }
            if (gamepad1.left_trigger > 0) {
                intakePow = -1;
                conveyorPow = -1;
            }

            //Button Controls
            if (gamepad1.x || gamepad2.x) {
                flyPow = 0;
                conveyorPow = 0;
            }
            if (gamepad1.y) {
                conveyorPow = 1;
            }

            //Bumper and Magnet Switch Controls
            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                robot.indexer.setPosition(0.25);
                conveyorPow = 1;
                flyPow = 1;
            } else if (robot.magnet.getState() == true) {
                robot.indexer.setPosition(0.25);
            } else {
                robot.indexer.setPosition(0.5);
            }

            //D-pad Controls
            if (gamepad1.dpad_up && linearPos < 0.9) {
                linearPos = linearPos + 0.001;
            }
            if (gamepad1.dpad_down && linearPos > 0.1) {
                linearPos = linearPos - 0.001;
            }

            //Set Powers
            robot.conveyor.setPower(motorPow(conveyorPow));
            robot.shooter2.setPower(motorPow(flyPow));
            robot.shooter1.setPower(motorPow(flyPow));
            robot.intake.setPower(motorPow(intakePow));
            robot.tilt.setPosition(linearPos);

            //Display the coordinates of the robot (inches)
            telemetry.addData("X", X);
            telemetry.addData("Y", Y);

            //Display the heading of the robot in degrees
            telemetry.addData("Data", Heading);
            telemetry.update();
        }

        t265Thread.interrupt();
        slamra.stop();
    }

    //Method for X-Drive
    public void mecDrive(double forward, double forward2, double turn, double strafe) {
        robot.lf.setPower(forward + forward2 + turn + strafe);
        robot.rf.setPower(forward + forward2 - turn - strafe);
        robot.lb.setPower(forward + forward2 + turn - strafe);
        robot.rb.setPower(forward + forward2 - turn + strafe);
    }

    //Method to get motor powers
    public double motorPow(double x){
        if(x>1){
            return x-1;
        }
        if(x<-1){
            return x+1;
        }
        return x;
    }

    //Method to get servo powers
    public double servoPow(double x){
        return (motorPow(x)*0.4) + 0.5;
    }

    //Method to Init SLAM and the camera with the correct starting coordinates
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

    //Thread Class
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

}



