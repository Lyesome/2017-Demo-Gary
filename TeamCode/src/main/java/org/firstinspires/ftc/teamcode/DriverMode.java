

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Lyesome on 2018-01-13.
 * This file contains all the instructions for controlling the robot in Teleop mode.
 */

@TeleOp(name="Driver Mode", group="Linear OpMode")  // @Autonomous(...) is the other common choice
//@Disabled
public class DriverMode extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    //creates a new robot named indianaGary
    private DcMotor motorDriveL = null;
    private DcMotor motorDriveR = null;
    private DcMotor motorShooterL = null;
    private DcMotor motorShooterR = null;
    private Servo servoLift = null;
    private Servo servoFlap = null;




    @Override
    public void runOpMode() {

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        //Provide warning for drivers not to hit play until initializing is complete.
        telemetry.addData("Status", "Initializing. Please Wait...");
        telemetry.update();

        motorDriveL = hardwareMap.dcMotor.get("motorDriveL");
        motorDriveL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorDriveR = hardwareMap.dcMotor.get("motorDriveR");
        motorDriveR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorShooterL = hardwareMap.dcMotor.get("motorShooterL");
        motorShooterL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorShooterR = hardwareMap.dcMotor.get("motorShooterR");
        motorShooterR.setDirection(DcMotorSimple.Direction.REVERSE);
        servoFlap = hardwareMap.servo.get("servoFlap");
        servoFlap.setDirection(Servo.Direction.FORWARD);
        servoLift = hardwareMap.servo.get("servoLift");
        servoLift.setDirection(Servo.Direction.FORWARD);
        //for manual driving encoder is not needed in the drive motors.

        //Set toggle initial states

        double FLAP_OPEN = 1.0;
        double FLAP_CLOSE = 0;
        long FLAP_WAIT = 3000;
        double SHOOTER_POWER = 0.2;

        double LIFT_RAISE = 1.0;
        double LIFT_LOWER = 0;
        long LIFT_WAIT = 2000;
        servoFlap.setPosition(FLAP_CLOSE);
        servoLift.setPosition(LIFT_LOWER);
        //tell drivers that initializing is now complete
        telemetry.setAutoClear(true);
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            //Drive System
            motorDriveL.setPower(gamepad1.left_stick_y);
            motorDriveR.setPower(gamepad1.left_stick_y);
            motorDriveL.setPower(gamepad1.right_stick_x);
            motorDriveR.setPower(-gamepad1.right_stick_x);
            if (gamepad1.x){
                servoFlap.setPosition(FLAP_OPEN);
                sleep(FLAP_WAIT);
                servoFlap.setPosition(FLAP_CLOSE);
            }
            if (gamepad1.right_trigger < 0.5 ){
                motorShooterR.setPower(SHOOTER_POWER);
                motorShooterL.setPower(SHOOTER_POWER);
                servoLift.setPosition(LIFT_RAISE);
                sleep(LIFT_WAIT);
                servoLift.setPosition(LIFT_LOWER);
                motorShooterR.setPower(0);
                motorShooterL.setPower(0);
            }

            motorShooterR.setPower(1);

        }
    }
}