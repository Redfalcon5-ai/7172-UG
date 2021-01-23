package org.firstinspires.ftc.teamcode.drive.Templates;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.RingDeterminationPipeline;
import org.firstinspires.ftc.teamcode.util.RobotHardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@TeleOp(name="RingDetector", group="LinearOpmode")

public class RingDetectorSample extends LinearOpMode {

    /* Declare OpMode members. */
    RobotHardware robot   = new RobotHardware();   // Use a Pushbot's hardware
    RingDeterminationPipeline rings   = new RingDeterminationPipeline();
    RingDeterminationPipeline.SkystoneDeterminationPipeline pipeline = new RingDeterminationPipeline.SkystoneDeterminationPipeline();
    OpenCvCamera webcam;


    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

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

        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();




        while (opModeIsActive())
        {
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Rings", pipeline.position);
            telemetry.update();
        }
    }

}


