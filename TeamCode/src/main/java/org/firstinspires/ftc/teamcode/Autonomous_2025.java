

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@Autonomous(name="YourBro", group="Robot")
//@Disabled
public class Autonomous_2025 extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor  leftDrive   = null; //the left drivetrain motor
    public DcMotor  rightDrive  = null; //the right drivetrain motor
    public DcMotor  armMotor    = null; //the arm motor
    public CRServo  intake      = null; //the active intake servo
    public Servo    wrist       = null; //the wrist servo


    /* This constant is the number of encoder ticks for each degree of rotation of the arm.
    To find this, we first need to consider the total gear reduction powering our arm.
    First, we have an external 20t:100t (5:1) reduction created by two spur gears.
    But we also have an internal gear reduction in our motor.
    The motor we use for this arm is a 117RPM Yellow Jacket. Which has an internal gear
    reduction of ~50.9:1. (more precisely it is 250047/4913:1)
    We can multiply these two ratios together to get our final reduction of ~254.47:1.
    The motor's encoder counts 28 times per rotation. So in total you should see about 7125.16
    counts per rotation of the arm. We divide that by 360 to get the counts per degree. */
    final double ARM_TICKS_PER_DEGREE =
            28 // number of encoder ticks per rotation of the bare motor
                    * 250047.0 / 4913.0 // This is the exact gear ratio of the 50.9:1 Yellow Jacket gearbox
                    * 100.0 / 20.0 // This is the external gear reduction, a 20T pinion gear that drives a 100T hub-mount gear
                    * 1/360.0; // we want ticks per degree, not per rotation


    /* These constants hold the position that the arm is commanded to run to.
    These are relative to where the arm was located when you start the OpMode. So make sure the
    arm is reset to collapsed inside the robot before you start the program.

    In these variables you'll see a number in degrees, multiplied by the ticks per degree of the arm.
    This results in the number of encoder ticks the arm needs to move in order to achieve the ideal
    set position of the arm. For example, the ARM_SCORE_SAMPLE_IN_LOW is set to
    160 * ARM_TICKS_PER_DEGREE. This asks the arm to move 160° from the starting position.
    If you'd like it to move further, increase that number. If you'd like it to not move
    as far from the starting position, decrease it. */

    final double ARM_COLLAPSED_INTO_ROBOT  = 0;
    final double ARM_COLLECT               = 250 * ARM_TICKS_PER_DEGREE;
    final double ARM_CLEAR_BARRIER         = 230 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SPECIMEN        = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_SCORE_SAMPLE_IN_LOW   = 160 * ARM_TICKS_PER_DEGREE;
    final double ARM_ATTACH_HANGING_HOOK   = 120 * ARM_TICKS_PER_DEGREE;
    final double ARM_WINCH_ROBOT           = 15  * ARM_TICKS_PER_DEGREE;

    /* Variables to store the speed the intake servo should be set at to intake, and deposit game elements. */
    final double INTAKE_COLLECT    = -1.0;
    final double INTAKE_OFF        =  0.0;
    final double INTAKE_DEPOSIT    =  0.5;

    /* Variables to store the positions that the wrist should be set to when folding in, or folding out. */
    final double WRIST_FOLDED_IN   = 0.8333;
    final double WRIST_FOLDED_OUT  = 0.52;

    /* A number in degrees that the triggers can adjust the arm position by */
    final double FUDGE_FACTOR = 15 * ARM_TICKS_PER_DEGREE;

    /* Variables that are used to set the arm to a specific position */
    double armPosition = (int)ARM_COLLAPSED_INTO_ROBOT;
    double armPositionFudgeFactor;

    /*Procedures for Autonoumous Mode */
    public void goForward (double power, long time) {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
        sleep(time);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void goRight(double power, long time) {
        leftDrive.setPower(power);
        rightDrive.setPower(-power);
        sleep(time);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
    public void goLeft(double power, long time) {
        leftDrive.setPower(-power);
        rightDrive.setPower(power);
        sleep(time);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void armToBucket() {
        armMotor.setTargetPosition((int) ARM_SCORE_SPECIMEN);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        while (opModeIsActive() && armMotor.isBusy()) {
            telemetry.addData("Arm Pos", armMotor.getCurrentPosition());
            telemetry.update();
        }
        sleep(500);
        wrist.setPosition(WRIST_FOLDED_OUT);
        sleep(1000);
        intake.setPower(INTAKE_DEPOSIT);
        sleep(1000);
        intake.setPower(INTAKE_OFF);
        armMotor.setTargetPosition((int)ARM_COLLAPSED_INTO_ROBOT);
        armMotor.setPower(1);
        while (opModeIsActive() && armMotor.isBusy()) {
            telemetry.addData("Arm Pos", armMotor.getCurrentPosition());
            telemetry.update();
        }
        armMotor.setPower(0);
    }

    @Override
    public void runOpMode() {
        /*
        These variables are private to the OpMode, and are used to control the drivetrain.
         */
        double left;
        double right;
        double forward;
        double rotate;
        double max;


        /* Define and Initialize Motors */
        leftDrive  = hardwareMap.get(DcMotor.class, "left_front_drive"); //the left drivetrain motor
        rightDrive = hardwareMap.get(DcMotor.class, "right_front_drive"); //the right drivetrain motor
        armMotor   = hardwareMap.get(DcMotor.class, "left_arm"); //the arm motor


        /* Most skid-steer/differential drive robots require reversing one motor to drive forward.
        for this robot, we reverse the right motor.*/
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);


        /* Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to slow down
        much faster when it is coasting. This creates a much more controllable drivetrain. As the robot
        stops much quicker. */
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        /*This sets the maximum current that the control hub will apply to the arm before throwing a flag */
        ((DcMotorEx) armMotor).setCurrentAlert(5,CurrentUnit.AMPS);


            /* Before starting the armMotor. We'll make sure the TargetPosition is set to 0.
            Then we'll set the RunMode to RUN_TO_POSITION. And we'll ask it to stop and reset encoder.
            If you do not have the encoder plugged into this motor, it will not run in this code. */
            armMotor.setTargetPosition(0);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        /* Define and initialize servos.*/
        intake = hardwareMap.get(CRServo.class, "intake");
        wrist  = hardwareMap.get(Servo.class, "wrist");

        /* Make sure that the intake is off, and the wrist is folded in. */
        intake.setPower(INTAKE_OFF);
        wrist.setPosition(WRIST_FOLDED_IN);

        /* Send telemetry message to signify robot waiting */
        telemetry.addLine("Robot Ready.");
        telemetry.update();

        /* Wait for the game driver to press play */
        waitForStart();

        /* Run until all code is complete */
        if (opModeIsActive()) {
            goForward(.5, 1550);
            goRight(.5, 700);
            goForward(.65, 2500);
            goLeft(.65, 350);
            goForward(.5, 350);
            armToBucket();

        }


    }
}
