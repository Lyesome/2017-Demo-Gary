package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Lyesome on 2018-01-03.
 */

@Autonomous(name="Indiana Gary - Red Left", group="Linear Opmode")
//@Disabled

public class Autonomous_R1 extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    BotConfig indianaGary = new BotConfig();
    private double jewelOffset = 0;
    private double columnOffset = 0;
    private double otf_correction = 0;
    private static double Drive_Power = 0.3;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initializing. Please Wait...");
        telemetry.update();
        indianaGary.InitAll(hardwareMap);

        String Team_Color = "red";

        while (!opModeIsActive()) {
            telemetry.addData("Status", "Initialized");
            telemetry.addData("OTF Correction (use D-pad to change)", otf_correction);
            telemetry.update();
            if (gamepad1.dpad_up)   { otf_correction = otf_correction + 0.1; }
            if (gamepad1.dpad_down) { otf_correction = otf_correction - 0.1; }

        }

        //waitForStart();
        runtime.reset();

        // run until the end of the match

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();


        //Autonomous Commands


        indianaGary.myGlyphLifter.Capture();
        columnOffset = indianaGary.myVuMark.DecodeImage(this);
        indianaGary.myJewelArm.LowerArm();
        jewelOffset = indianaGary.myJewelArm.JewelKnock("blue");

        indianaGary.drive.Drive(this, Drive_Power, jewelOffset);

        indianaGary.myJewelArm.RaiseArm();

        indianaGary.drive.Drive(this, Drive_Power, 36 - jewelOffset + columnOffset + otf_correction);

        indianaGary.drive.Turn(this, -90);

        indianaGary.drive.Drive(this, Drive_Power, 2);

        indianaGary.myGlyphLifter.Release();
        indianaGary.myGlyphLifter.GotoPresetPosition(0);
        sleep(1000);
        //indianaGary.drive.Turn(this,180);
        //indianaGary.myGlyphLifter.init(hardwareMap);
        //indianaGary.drive.Drive(this, Drive_Power, -1);

    }


}
