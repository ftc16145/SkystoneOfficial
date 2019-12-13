package org.firstinspires.ftc.teamcode.utils;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Map;

public class PointCalculator {
    public enum target{
        redDepotForward,blueDepotForward,redDepotSide,blueDepotSide,redFound18,blueFound18,redFoundOrig,blueFoundOrig,stone
    }
    private static final float mmPerInch = 25.4f;
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    VuforiaTrackable stoneTarget, blueRearBridge, redRearBridge, redFrontBridge, blueFrontBridge, red1, red2, front1, front2, blue1, blue2, rear1, rear2;
    private Map<VuforiaTrackable,double[]> redDepotForward;
    private Map<VuforiaTrackable,double[]> blueDepotForward;
    private Map<VuforiaTrackable,double[]> redDepotSide;
    private Map<VuforiaTrackable,double[]> blueDepotSide;
    private Map<VuforiaTrackable,double[]> redFound;
    private Map<VuforiaTrackable,double[]> blueFound;
    private Map<VuforiaTrackable,double[]> skystone;
    public PointCalculator(VuforiaTrackables targetsSkyStone){
        defineAllTargets(targetsSkyStone);
        skystone.put(stoneTarget, new double[]{-12,0});

    }
    public double[] getTargetPoint( VuforiaTrackable vuf, target t ){
        try{
            switch(t){
                case redDepotForward:
                    return redDepotForward.get( vuf );
                case blueDepotForward:
                    return blueDepotForward.get( vuf );
                case redDepotSide:
                    return redDepotSide.get( vuf );
                case blueDepotSide:
                    return blueDepotSide.get( vuf );
                case redFoundOrig:
                    return redFound.get( vuf );
                case blueFoundOrig:
                    return blueFound.get( vuf );
                case redFound18:
                    double[] tempR = redFound.get( vuf );
                    return new double[]{tempR[0],tempR[1]-30};
                case blueFound18:
                    double[] tempB = blueFound.get( vuf );
                    return new double[]{tempB[0],tempB[1]-30};
                case stone:
                    return skystone.get( vuf );
                default:
                    return null;
            }
        }catch( Exception e ){
            return null;
        }
    }
    private void defineAllTargets(VuforiaTrackables targetsSkyStone){
        stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");
    }
}
