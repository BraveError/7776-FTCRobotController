package org.firstinspires.ftc.teamcode;

import static java.lang.System.currentTimeMillis;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.DeadWheelDirectionDebugger;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous
public class AutoTest2 extends LinearOpMode {
//    private Drive DriveController;
    private Intake IntakeController;
    private DecoderWheel DecoderWheelController;
    private OutTake OutTakeController;

    @Override
    public void runOpMode() {
        Drive.telemetry = telemetry;

        waitForStart();
        if (!opModeIsActive()) {
            return;
        }

        Servo InLeftServo = hardwareMap.get(Servo.class, "intakelefts");
        Servo InRightServo = hardwareMap.get(Servo.class, "intakerights");

        DcMotor InMotor = hardwareMap.get(DcMotor.class, "intake");

        this.IntakeController = new Intake();
        this.IntakeController.Init(InLeftServo, InRightServo, InMotor);

        DcMotor DecoderWheelMotor = hardwareMap.get(DcMotor.class, "ringdrive");

        this.DecoderWheelController = new DecoderWheel();
        this.DecoderWheelController.Init(DecoderWheelMotor);

        this.DecoderWheelController.SetIntake(IntakeController);

        DcMotorEx OutLeft = (DcMotorEx) hardwareMap.get(DcMotor.class, "outl");
        DcMotorEx OutRight = (DcMotorEx) hardwareMap.get(DcMotor.class, "outr");

        Servo OutLeftServo = hardwareMap.get(Servo.class, "outservol");
        Servo OutRightServo = hardwareMap.get(Servo.class, "outservor");

        this.OutTakeController = new OutTake();
        this.OutTakeController.Init(OutLeft, OutRight, OutLeftServo, OutRightServo);

        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(new ParallelAction(
                    this.DecoderWheelController.AutoStartUpdateLoop(),
                    new SequentialAction(
                        this.DecoderWheelController.AutoIntakeModeOn(),
                        this.IntakeController.AutoStartIntaking(),
                        drive.actionBuilder(beginPose)
                                .splineTo(new Vector2d(28, 0), 0)
                                .build(),
                        new SleepAction(0.5),
                        this.DecoderWheelController.AutoRevolveLeft(),
                        this.DecoderWheelController.AutoIntakeModeOff(),
                        this.IntakeController.AutoStartIntakingForRevolve(),
                        this.DecoderWheelController.AutoIntakeModeOn(),
                        new SleepAction(0.5),
                        this.IntakeController.AutoStartIntaking(),
                        drive.actionBuilder(beginPose.plus(new Twist2d(new Vector2d(28, 0), 0)))
                                .turn(Math.PI / 2)
                                .lineToY(24)
                                .build(),
                        new SleepAction(0.5),
                        this.IntakeController.AutoStopIntaking(),
                        this.DecoderWheelController.AutoIntakeModeOff(),
                        drive.actionBuilder(beginPose)
                                .splineToSplineHeading(new Pose2d(0.1, 0.1, 0), 0)
                                .build(),
                        this.OutTakeController.AutoSpinUp(),
                        this.IntakeController.AutoStopIntaking(),
                        new SleepAction(4),
                        this.OutTakeController.AutoServosUp(),
                        new SleepAction(0.2),
                        this.OutTakeController.AutoServosDown(),
                        new SleepAction(0.2),
                        this.DecoderWheelController.AutoRevolveLeft(),
                        this.IntakeController.AutoStartIntakingForRevolve(),
                        new SleepAction(0.6),
                        this.IntakeController.AutoStopIntaking(),
                        this.OutTakeController.AutoServosUp(),
                        new SleepAction(0.2),
                        this.OutTakeController.AutoServosDown(),
                        this.OutTakeController.AutoSpinDown()
            )));
        } else {
            throw new RuntimeException();
        }
    }
}
