package com.example.meepmeeptesting.red.wing.right;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class Auton14_Right {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity realBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(15, 18)
                .setColorScheme(new ColorSchemeRedDark())
                .setConstraints(43.857278219984295, 43.857278219984295, 4.64, 4.64, 0.425)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -64.5, Math.toRadians(180)))
                                .strafeRight(6)
                                .splineToSplineHeading(new Pose2d(-48, -36, Math.toRadians(90)), Math.toRadians(0))
                                .splineToSplineHeading(new Pose2d(-31, -36, Math.toRadians(0)), Math.toRadians(0))
                                .back(10)
                                .setReversed(true)
                                .splineToConstantHeading(new Vector2d(-36, -60), Math.toRadians(0))
                                .forward(48)
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(50, -42), Math.toRadians(0))
                                .strafeLeft(30)
                                .forward(10)
                                .build()
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(realBot)
                .start();
    }
}