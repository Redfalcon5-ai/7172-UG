package org.firstinspires.ftc.teamcode.drive.PVC;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.spartronics4915.lib.T265Camera;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.RingDeterminationPipeline;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name="PVCRedAuton", group="Linear Opmode")
public class PVCRedAuton extends LinearOpMode
{
    //Create elapsed time and robot hardware objects
    RobotHardware robot   = new RobotHardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    //Define ticks per inch
    static final double     tickInch = 34.22 ;

    //Create T265 object
    private static T265Camera slamra = null;

    //Init var for camera pos on robot (mount with cable on left side)
    String CameraPos = "left";
    String left = "left";
    String right = "right";
    String back = "back";
    String front = "front";

    //Vars for T265 multi-threading
    double Y = 0;
    double X = 0;
    double Heading = 0;

    //OpenCV stuff
    RingDeterminationPipeline rings   = new RingDeterminationPipeline();
    RingDeterminationPipeline.SkystoneDeterminationPipeline pipeline = new RingDeterminationPipeline.SkystoneDeterminationPipeline();
    OpenCvCamera webcam;

    @Override
    public void runOpMode() {
        //Init hardware map and set motor directions + servo postions
        robot.init(hardwareMap);
        robot.rf.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rb.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lf.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lb.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.intake.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.grabber.setPosition(0.025);
        robot.tilt.setPosition(0.56);

        //Start OpenCV
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new RingDeterminationPipeline.SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
                                     {
                                         @Override
                                         public void onOpened()
                                         {

                                             webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                                         }
                                     }
        );

        //Var for rings in starter stack
        int rings = 0;

        //Set motors to encoder modes
        initEncoders();

        //Create T265 thread
        Thread t265Thread = new T265Thread();

        //Initialize camera
        initCameraPos();

        //Start Camera
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

        //Start updating pos
        t265Thread.start();

        //Aim and bring flywheel up to speed
        robot.shooter1.setVelocity(2300);
        robot.shooter2.setVelocity(2300);
        robot.intake.setPower(1);
        robot.conveyor.setPower(1);
        sleep(2000);

        //Shot 1
        robot.indexer.setPosition(0.25);
        sleep(200);
        boolean ringShot = false;
        while(ringShot == false){
            if(robot.magnet.getState() == true){
                robot.indexer.setPosition(0.25);
            }
            else if (robot.magnet.getState() == false){
                robot.indexer.setPosition(0.5);
                ringShot = true;
            }
        }

        //Wait for wheel to regain speed
        sleep(1000);

        //Shot 2
        robot.indexer.setPosition(0.25);
        sleep(200);
        ringShot = false;
        while(ringShot == false){
            if(robot.magnet.getState() == true){
                robot.indexer.setPosition(0.25);
            }
            else if (robot.magnet.getState() == false){
                robot.indexer.setPosition(0.5);
                ringShot = true;
            }

        }

        //Wait for wheel to regain speed
        sleep(1000);

        //Shot 3
        robot.indexer.setPosition(0.25);
        sleep(200);
        ringShot = false;
        while(ringShot == false){
            if(robot.magnet.getState() == true){
                robot.indexer.setPosition(0.25);
            }
            else if (robot.magnet.getState() == false){
                robot.indexer.setPosition(0.5);
                ringShot = true;
            }

        }

        //Wait for 1 more second to ensure third ring has been shot
        sleep(1000);
        robot.shooter1.setVelocity(0);
        robot.shooter2.setVelocity(0);
        robot.intake.setPower(0);
        robot.conveyor.setPower(0);

        //Determine the number of rings in the starter stack
        if(pipeline.position == RingDeterminationPipeline.SkystoneDeterminationPipeline.RingPosition.FOUR){
            rings = 4;
        }
        else if(pipeline.position == RingDeterminationPipeline.SkystoneDeterminationPipeline.RingPosition.ONE){
            rings = 1;
        }
        else if(pipeline.position == RingDeterminationPipeline.SkystoneDeterminationPipeline.RingPosition.NONE){
            rings = 0;
        }

        //Deliver wobble goal accordingly
        if(rings == 4){
            encoderDrive(0.5, 30, -30, -30, 30, 5);
            encoderDrive(0.5, 110, 110, 110, 110, 5);
            sleep(1000);
            robot.grabber.setPosition(0.5);
            sleep(1000);
            encoderDrive(0.5, -30, -30, -30, -30, 5);

        }

        if(rings == 0){
            encoderDrive(0.5, 30, -30, -30, 30, 5);
            encoderDrive(0.5, 50, 50, 50, 50, 5);
            sleep(1000);
            robot.grabber.setPosition(0.5);
            sleep(1000);
            encoderDrive(0.5, -35, 35, 35, -35, 5);
            encoderDrive(0.5, 7, 7, 7, 7, 5);

        }

        if(rings == 1){
            encoderDrive(0.5, 30, -30, -30, 30, 5);
            encoderDrive(0.5, 85, 85, 85, 85, 5);
            encoderDrive(0.5, -20, 20, 20, -20, 5);
            sleep(1000);
            robot.grabber.setPosition(0.5);
            sleep(1000);
            encoderDrive(0.5, -10, -10, -10, -10, 5);

        }

        while(opModeIsActive()){
            telemetry.addData("Heading", Heading);
            telemetry.addData("X", X);
            telemetry.addData("Y", Y);
            telemetry.update();
        }

        t265Thread.interrupt();

        slamra.stop();


    }

    //Thread for updating pos
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
                    Heading = (up.pose.getHeading()) * (-57.295);
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

    //Init camera pos on the field
    public void initCameraPos(){
        for(int i = 0; i < 2; i++) {
            if (slamra == null) {
                slamra = new T265Camera(new Transform2d(), 0.1, hardwareMap.appContext);
            }
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

    //Go to position using encoder values
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

    //Set motors to encoder modes
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



}

