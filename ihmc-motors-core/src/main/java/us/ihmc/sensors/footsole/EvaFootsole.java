package us.ihmc.sensors.footsole;

import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.awt.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

public class EvaFootsole
{
    private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

    private final List<Point2D> sensorPointsInSoleFrame = new ArrayList<Point2D>();
    private static final double sensorLength = 0.02;
    private static final double sensorWidth = 0.02;

    private final double footLength; //TODO update with foot sole length
    private final double footWidth; //TODO update with foot sole width

    private final ArrayList<YoFrameConvexPolygon2D> yoPressureSpaces = new ArrayList<>();
    private final FrameConvexPolygon2D tmpSensorPolygon = new FrameConvexPolygon2D();

    private Footstep footstep;
    private final List<FramePoint2D> contactPointsInSoleFrame2d = new ArrayList<FramePoint2D>();

    private final YoFramePoseUsingYawPitchRoll footPose = new YoFramePoseUsingYawPitchRoll("FootPose", worldFrame, registry);
    private final ReferenceFrame soleFrame;
    private final ReferenceFrame soleZUpFrame;

    private final FrameConvexPolygon2D footstepPolygon = new FrameConvexPolygon2D();
    private final FrameConvexPolygon2D tempFootstepPolygonForShrinking = new FrameConvexPolygon2D();
    private final ConvexPolygonScaler convexPolygonShrinker = new ConvexPolygonScaler();
    private final YoFramePoseUsingYawPitchRoll yoNextFootstepPose = new YoFramePoseUsingYawPitchRoll("nextFootstepPose", worldFrame, registry);
    private final YoFrameConvexPolygon2D yoNextFootstepPolygon = new YoFrameConvexPolygon2D("nextFootstep", "", worldFrame, 4, registry);

    private HashMap<Integer, YoDouble> pressureSensorValueMap = new HashMap<>();

    private final YoFramePoint2D yoMeasuredCoP = new YoFramePoint2D("measuredCoP", worldFrame, registry);

    private final YoGraphicsListRegistry yoGraphicsListRegistry;

    public EvaFootsole(double footLength, double footWidth,
                       YoRegistry parentRegistry,
                       YoGraphicsListRegistry yoGraphicsListRegistry)
    {

        this.footLength = footLength;
        this.footWidth = footWidth;
        this.yoGraphicsListRegistry = yoGraphicsListRegistry;

        createFootstep();
        initializePressureSensorShape();
        initializePressureSensorCentroids();
        setupFootsole();

        this.soleFrame = footstep.getSoleReferenceFrame();
        this.soleZUpFrame = new ZUpFrame(worldFrame, footstep.getSoleReferenceFrame(), "ZUp");

        YoGraphicPosition measuredCoPGraphic = new YoGraphicPosition("measuredCoP", yoMeasuredCoP, 0.01, YoAppearance.Red(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
        yoGraphicsListRegistry.registerYoGraphic("measuredCoP", measuredCoPGraphic);
        yoGraphicsListRegistry.registerArtifact("measuredCoP", measuredCoPGraphic.createArtifact());

        parentRegistry.addChild(registry);
    }

    private void createFootstep()
    {
        List<Point2D> contactPointsInSoleFrame = new ArrayList<Point2D>();
        contactPointsInSoleFrame.add(new Point2D(footLength / 2.0, footWidth / 2.0));
        contactPointsInSoleFrame.add(new Point2D(footLength / 2.0, -footWidth / 2.0));
        contactPointsInSoleFrame.add(new Point2D(-footLength / 2.0, -footWidth / 2.0));
        contactPointsInSoleFrame.add(new Point2D(-footLength / 2.0, footWidth / 2.0));
        FramePose3D startingPose = new FramePose3D();

        footstep = new Footstep(RobotSide.LEFT, startingPose);
        footstep.setPredictedContactPoints(contactPointsInSoleFrame);

        for (Point2D contactPointInSoleFrame : contactPointsInSoleFrame)
        {
            FramePoint3D point = new FramePoint3D(footstep.getSoleReferenceFrame(), contactPointInSoleFrame.getX(), contactPointInSoleFrame.getY(), 0.0);
            contactPointsInSoleFrame2d.add(new FramePoint2D(point));
        }

        Graphics3DObject footGraphics = new Graphics3DObject();
        footGraphics.addExtrudedPolygon(footstep.getPredictedContactPoints(), 0.02, YoAppearance.Blue());
        yoGraphicsListRegistry.registerYoGraphic("FootViz", new YoGraphicShape("FootViz", footGraphics, footPose, 1.0));

        yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("nextFootstep", yoNextFootstepPolygon, Color.blue, false));

    }

    private void initializePressureSensorShape()
    {
        sensorPointsInSoleFrame.add(new Point2D(sensorLength, sensorWidth));
        sensorPointsInSoleFrame.add(new Point2D(sensorLength, -sensorWidth));
        sensorPointsInSoleFrame.add(new Point2D(-sensorLength, -sensorWidth));
        sensorPointsInSoleFrame.add(new Point2D(-sensorLength, sensorWidth));
        tmpSensorPolygon.setIncludingFrame(soleFrame, Vertex2DSupplier.asVertex2DSupplier(sensorPointsInSoleFrame));
    }

    private void initializePressureSensorCentroids()
    {
        addCentroid(0, -0.02652, -0.02389);
        addCentroid(1, 0.03448 , -0.02511);
        addCentroid(2, 0.08634 , -0.03239);
        addCentroid(3, 0.09356 , 0.01633);
        addCentroid(4, 0.03656 , 0.02409);
        addCentroid(5, -0.03452, 0.02498);
        addCentroid(6, -0.09421, 0.01821);
        addCentroid(7, -0.09118, -0.03123);
        for(int i = 0; i < yoPressureSpaces.size(); i++)
            pressureSensorValueMap.put(i, new YoDouble("pressureCoP"+i, registry));
    }

    private void addCentroid(int sensorNumber, double x, double y)
    {
        tmpSensorPolygon.setIncludingFrame(worldFrame, Vertex2DSupplier.asVertex2DSupplier(sensorPointsInSoleFrame));
        tmpSensorPolygon.translate(x, y);
        tmpSensorPolygon.changeFrameAndProjectToXYPlane(worldFrame);
        yoPressureSpaces.add(new YoFrameConvexPolygon2D("pressurePoint"+sensorNumber, "", worldFrame, 4, registry));
        yoPressureSpaces.get(sensorNumber).set(tmpSensorPolygon);
    }

    private void setupFootsole()
    {
        double polygonShrinkAmount = 0.0;

        tempFootstepPolygonForShrinking.setIncludingFrame(footstep.getSoleReferenceFrame(), Vertex2DSupplier.asVertex2DSupplier(footstep.getPredictedContactPoints()));
        convexPolygonShrinker.scaleConvexPolygon(tempFootstepPolygonForShrinking, polygonShrinkAmount, footstepPolygon);

        footstepPolygon.changeFrameAndProjectToXYPlane(worldFrame);
        yoNextFootstepPolygon.set(footstepPolygon);

        FramePose3D nextFootstepPose = new FramePose3D(footstep.getSoleReferenceFrame());
        yoNextFootstepPose.setMatchingFrame(nextFootstepPose);

        for(int i = 0; i < yoPressureSpaces.size(); i++)
            yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("pressurePoint"+i, yoPressureSpaces.get(i), Color.red, false));

    }

    public void updateCoP(double[] measuredNormalForces)
    {
        for(int sensor = 0; sensor < yoPressureSpaces.size(); sensor++)
        {
            setNormalForce(sensor, measuredNormalForces[sensor]);
        }
        updateCoP();
    }

    public void setNormalForce(int sensorNumber, double normalForce)
    {
        pressureSensorValueMap.get(sensorNumber).set(normalForce);
    }

    public void updateCoP()
    {
        double centerOfPressureX = 0.0, centerOfPressureY = 0.0;
        double force;
        double distanceX, distanceY;
        double sumForce = 0.0;
        for(int pressureSensor = 0; pressureSensor < yoPressureSpaces.size(); pressureSensor++)
        {
            force = pressureSensorValueMap.get(pressureSensor).getDoubleValue();
            distanceX = yoPressureSpaces.get(pressureSensor).getCentroid().getX();
            distanceY = yoPressureSpaces.get(pressureSensor).getCentroid().getY();

            centerOfPressureX += force * distanceX;
            centerOfPressureY += force * distanceY;
            sumForce += force;
        }
        centerOfPressureX /= sumForce;
        centerOfPressureY /= sumForce;
        yoMeasuredCoP.set(centerOfPressureX, centerOfPressureY);
    }

}