package org.firstinspires.ftc.teamcode.drive.Templates;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.DualPad;
import org.firstinspires.ftc.teamcode.util.RobotHardware;

@TeleOp(name="mecDriveBaseDual", group="Linear Opmode")
public class mecDriveBaseDual extends LinearOpMode
{
    //Create Robot Hardware Object
    RobotHardware robot = new RobotHardware();

    DualPad gpad = new DualPad();

    //Create Variables for Motor/Servo Powers
    double linearPos = 0.564;
    double intakePow = 0.0;
    double conveyorPow = 0.0;
    double flyPow = 0.0;
    double grabberPow = 0.0;

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
            gpad.mergePads(gamepad1, gamepad2);

            //Driving Controls
            double leftStickY = -gpad.left_stick_y; //Forward and Backward
            double rightStickY = -gpad.right_stick_y;   //Forward and Backward
            double leftStickX = gpad.left_stick_x;  //Turning
            double rightStickX = gpad.right_stick_x;    //Strafing
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
            if (true) {
                grabberPow = 0;
            }


            //Distance Sensor controls
            if (((DistanceSensor) robot.colorv3).getDistance(DistanceUnit.CM) < 4) {
                flyPow = 2300;
                conveyorPow = 2;
            }



            //Button Controls

            if (gpad.a) {
                robot.grabber.setPosition(0.025);
            }
            if (gpad.aShift) {
                grabberPow = 1000;
            }
            if (gpad.b) {
                robot.grabber.setPosition(0.5);
            }
            if (gpad.bShift) {
                grabberPow = -1000;
            }

            //Bumper and Magnet Switch Controls
            if (gpad.right_bumperShift) {
                robot.indexer.setPosition(0.75);
            }
            else if (gpad.right_bumper) {
                robot.indexer.setPosition(0.25);
            }
            else if (robot.magnet.getState() == false) {
                robot.indexer.setPosition(0.5);
            }



            //D-pad Controls
            if (gpad.dpad_up && linearPos < 0.9) {
                linearPos = linearPos + 0.001;
            }
            if (gpad.dpad_down && linearPos > 0.1) {
                linearPos = linearPos - 0.001;
            }
            if (gpad.dpad_left) {
                linearPos = 0.584;
            }
            if (gpad.dpad_right) {
                linearPos = 0.564;
            }

            //Unlatch
            if (gpad.x) {
                flyPow = 0;
            }
            if (gpad.xShift) {
                flyPow = 2300;
                conveyorPow = 2;
            }

            //Trigger Controls
            if (gamepad1.right_trigger > 0) {
                intakePow = 1;
                if(conveyorPow != 2){
                    conveyorPow = 1;
                }
            }
            if (gamepad1.left_trigger > 0) {
                intakePow = -1;
                if(conveyorPow != 2){
                    conveyorPow = -1;
                }
            }

            //Set Powers
            robot.conveyor.setPower(motorPow(conveyorPow));
            robot.shooter2.setVelocity(flyPow);
            robot.shooter1.setVelocity(flyPow);
            robot.intake.setPower(motorPow(intakePow));
            robot.wobble.setVelocity(grabberPow);
            robot.tilt.setPosition(linearPos);

            telemetry.addData("TiltPos", linearPos);
            telemetry.addData("Shooter1", robot.shooter1.getVelocity());
            telemetry.addData("Shooter2", robot.shooter2.getVelocity());

            telemetry.update();
        }
    }

    //Method for Mecanum Drive
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

}




