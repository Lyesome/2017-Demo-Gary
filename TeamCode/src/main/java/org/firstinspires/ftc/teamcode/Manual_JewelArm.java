package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Lyesome on 2018-01-03.
 */
@TeleOp(name="Manual Jewel Arm", group="Linear Opmode")
//@Disabled

public class Manual_JewelArm extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor motorFL = null;
    private DcMotor motorFR = null;
    private DcMotor motorBL = null;
    private DcMotor motorBR = null;
    private static double Drive_Power = 0.5;

    private DcMotor motorLift = null;
    private Servo GrabberR = null;
    private Servo GrabberL = null;

    private ColorSensor colorSensorF;    // Hardware Device Object
    private ColorSensor colorSensorB;
    private Servo JewelArm = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        motorFL = hardwareMap.dcMotor.get("motor_fl");
        //motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR = hardwareMap.dcMotor.get("motor_fr");
        //motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL = hardwareMap.dcMotor.get("motor_bl");
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR = hardwareMap.dcMotor.get("motor_br");
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        colorSensorF = hardwareMap.get(ColorSensor.class, "sensor_colorF");
        colorSensorB = hardwareMap.get(ColorSensor.class, "sensor_colorB");
        JewelArm = hardwareMap.servo.get("JewelArm");

        motorLift  = hardwareMap.dcMotor.get("glyph_lifter");
        GrabberL = hardwareMap.servo.get("Glyph_Pad_Left");
        GrabberR = hardwareMap.servo.get("Glyph_Pad_Right");
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // eg: Set the drive motor directions:
        // "Reverse" the motor that runs backwards when connected directly to the battery
        motorFL.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorFR.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        motorBL.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        motorBR.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        motorLift.setDirection(DcMotor.Direction.REVERSE);

        colorSensorF.enableLed(false);
        colorSensorB.enableLed(false);

        String Team_Color = "red";
        Double JewelOffset;
        Double ColumnOffset;

        //JewelArm.setPosition(0.37);

        waitForStart();
        runtime.reset();

        // run until the end of the match
        while(opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();


            //Autonomous Commands
            //GlyphCapture();
            //GlyphRelease();
            telemetry.addData("Jewel Arm @ ", JewelArm.getPosition());
            telemetry.update();
            JewelArm.setPosition(gamepad2.right_trigger);

        }

        //JewelOffset = JewelKnock(Team_Color);
    }


    private void StopWheels() {
        telemetry.addData("Wheels: ", "Stopped");
        telemetry.update();
        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        sleep(500);
    }


    private void DriveForward(double power, double distance) {
        //Drive forward distance in inches. Use "scaleFactor" to convert inches to encoder values.
        double scaleFactor = 89.17;
        double startPosition = motorBL.getCurrentPosition();
        double endPosition = (startPosition + (distance * scaleFactor));
        telemetry.addData("Driving Forward: ", Double.toString(distance) + " inches");
        telemetry.addData("Start Position: ", startPosition);
        telemetry.addData("End Position: ", endPosition);
        telemetry.update();

        while (motorBL.getCurrentPosition() < endPosition) {
            motorFL.setPower(power);
            motorFR.setPower(power);
            motorBL.setPower(power);
            motorBR.setPower(power);
            telemetry.addData("Distance Remaining: ", Double.toString(scaleFactor * (endPosition - motorBL.getCurrentPosition())) + " inches");
            telemetry.update();
        }
        StopWheels();
        telemetry.addData("Destination ", "Reached");
        telemetry.update();
    }

    private void DriveBackward(double power, double distance) {
        //Drive backwards distance in inches. Use "scaleFactor" to convert inches to encoder values.
        double scaleFactor = 89.17;
        double startPosition = motorBL.getCurrentPosition();
        double endPosition = (startPosition - (distance * scaleFactor));
        telemetry.addData("Driving Backward: ", Double.toString(distance) + " inches");
        telemetry.addData("Start Position: ", startPosition);
        telemetry.addData("End Position: ", endPosition);
        telemetry.update();
        while (motorBL.getCurrentPosition() > endPosition) {
            motorFL.setPower(-power);
            motorFR.setPower(-power);
            motorBL.setPower(-power);
            motorBR.setPower(-power);
            telemetry.addData("Distance Remaining: ", Double.toString(scaleFactor * (motorBL.getCurrentPosition() - endPosition)) + " inches");
            telemetry.update();
        }
        StopWheels();
        telemetry.addData("Destination ", "Reached");
        telemetry.update();
    }

    private void DriveRight(double power, double distance) {
        //Drive backwards distance in inches. Use "scaleFactor" to convert inches to encoder values.
        double scaleFactor = 178.34;
        double startPosition = motorBL.getCurrentPosition();
        double endPosition = (startPosition - (distance * scaleFactor));
        telemetry.addData("Driving Right: ", Double.toString(distance) + " inches");
        telemetry.addData("Start Position: ", startPosition);
        telemetry.addData("End Position: ", endPosition);
        telemetry.update();
        while (motorBL.getCurrentPosition() > endPosition) {
            motorFL.setPower(power);
            motorFR.setPower(-power);
            motorBL.setPower(-power);
            motorBR.setPower(power);
            telemetry.addData("Distance Remaining: ", Double.toString(scaleFactor * (motorBL.getCurrentPosition() - endPosition)) + " inches");
            telemetry.update();
        }
        StopWheels();
        telemetry.addData("Destination ", "Reached");
        telemetry.update();
    }
    private void DriveLeft(double power, double distance) {
        //Drive backwards distance in inches. Use "scaleFactor" to convert inches to encoder values.
        double scaleFactor = 178.34;
        double startPosition = motorBL.getCurrentPosition();
        double endPosition = (startPosition + (distance * scaleFactor));
        telemetry.addData("Driving Left: ", Double.toString(distance) + " inches");
        telemetry.addData("Start Position: ", startPosition);
        telemetry.addData("End Position: ", endPosition);
        telemetry.update();
        while (motorBL.getCurrentPosition() < endPosition) {
            motorFL.setPower(-power);
            motorFR.setPower(power);
            motorBL.setPower(power);
            motorBR.setPower(-power);
            telemetry.addData("Distance Remaining: ", Double.toString(scaleFactor * (endPosition - motorBL.getCurrentPosition())) + " inches");
            telemetry.update();
        }
        StopWheels();
        telemetry.addData("Destination ", "Reached");
        telemetry.update();
    }

    private void GlyphCapture() {
        GrabberL.setPosition(1.0);
        GrabberR.setPosition(0.0);
        motorLift.setPower(0.2);
        sleep(1000);
        motorLift.setPower(0.0);
    }

    private void GlyphRelease(){
        GrabberL.setPosition(0.8);
        GrabberR.setPosition(0.2);
    }

    private double JewelKnock(String myColor){
        //Knock off other team's jewel by driving foward or backwards based on color sensor
        //Return distance travelled in inches
        double Move_Distance = 0;
        //Lower jewel arm
        JewelArm.setPosition(0.31);
        //Turn on LEDs
        colorSensorF.enableLed(true);
        colorSensorB.enableLed(true);
        //Give time for jewel arm to move and color sensor to read values
        sleep(1000);
        //Knock off the jewel
        if (ColorRedTestFront() == 0) {
            Move_Distance = 0;
        }
        if (ColorRedTestFront() == 1) {
            Move_Distance = -2;
        }
        if (ColorRedTestFront() == 2) {
            Move_Distance = 2;
        }
        DriveForward(Drive_Power, Move_Distance);

        //Raise jewel arm
        JewelArm.setPosition(0.37);
        //Turn off LEDs
        colorSensorF.enableLed(false);
        colorSensorB.enableLed(false);

        return Move_Distance;
    }

    private int ColorRedTestFront() {
        int result = 0;
        if (colorSensorF.red() == colorSensorB.red()) {
            result = 0;
        }
        if (colorSensorF.red() > colorSensorB.red()) {
            result = 1;
        }
        if (colorSensorF.red() < colorSensorB.red()) {
            result = 2;
        }
        return result;
    }

}