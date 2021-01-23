package org.firstinspires.ftc.teamcode.drive.Templates;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

@Disabled
@TeleOp(name="mecDriveEncoderTurn", group="Linear Opmode")
public class mecDriveEncoderTurn extends LinearOpMode
{
    //Create Robot Hardware and Elapsed Time Objects
    RobotHardware robot = new RobotHardware();
    private ElapsedTime runtime = new ElapsedTime();

    //Create Variables for Motor/Servo Powers
    double linearPos = 0.5;
    double intakePow = 0.0;
    double conveyorPow = 0.0;
    double flyPow = 0.0;

    //34.22 Ticks for 1 inch
    static final double     tickInch = 34.22;


    //Code to run on Init
    @Override
    public void runOpMode() {
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

        waitForStart();

        while(opModeIsActive()) {
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

            //Turning
            if (gamepad1.a) {
                initEncoder();
                for (int i = 0; i < 1; i++)
                    encoderDrive(0.5, -1, 1, -1, 1, 5.0);

                //1 inch = 3.225 degrees
            }

            telemetry.update();
        }
    }

    //Method for X-Drive
    public void mecDrive(double forward, double forward2, double turn, double strafe) {
        robot.lf.setPower(forward + forward2 + turn + strafe);
        robot.rf.setPower(forward + forward2 - turn - strafe);
        robot.lb.setPower(forward + forward2 + turn - strafe);
        robot.rb.setPower(forward + forward2 - turn + strafe);
    }

    //Method for driving via encoders
    public void encoderDrive(double speed, double lfInches, double rfInches, double lbInches, double rbInches, double timeoutS) {
        int newlFTarget;
        int newrFTarget;
        int newlBTarget;
        int newrBTarget;

        // Ensure that the opmode is still active
        robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        while ((runtime.seconds() < timeoutS) && (robot.lf.isBusy() && robot.rf.isBusy() && robot.lb.isBusy() && robot.rb.isBusy())) {

            // Display it for the driver.
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

        /*
        robot.lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        */

    }

    //Init encoders
    public void initEncoder(){
        robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        robot.lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
}


