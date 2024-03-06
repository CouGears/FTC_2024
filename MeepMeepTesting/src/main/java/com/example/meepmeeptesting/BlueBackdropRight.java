package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueBackdropRight {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(750);

        RoadRunnerBotEntity realBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(15, 18)
                .setColorScheme(new ColorSchemeBlueDark())
                .setConstraints(43.857278219984295, 43.857278219984295, 4.64, 4.64, 0.425)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, 64.5, 0))
                                .splineToLinearHeading(new Pose2d(36, 48, Math.toRadians(270)), Math.toRadians(0))
                                .splineToLinearHeading(new Pose2d(0, 36, Math.toRadians(180)), Math.toRadians(0))
                                .setReversed(true)
                                .splineToLinearHeading(new Pose2d(24, 60, Math.toRadians(0)), Math.toRadians(0))
                                .setReversed(false)
                                .lineToConstantHeading(new Vector2d(48, 30))
                                .strafeLeft(30)
                                .forward(12)
                                .build()
                );



        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(realBot)
                .start();
    }
}