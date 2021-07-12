package us.ihmc.sensors.footsole;

import us.ihmc.commonWalkingControlModules.controllers.Updatable;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicShape;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.awt.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

public class FootsoleCoPPlotter
{
   private static final int BUFFER_SIZE = 16000;
   private final double dt = 0.006;
   private final ArrayList<Updatable> updatables = new ArrayList<>();

   private final RobotSide robotSide = RobotSide.LEFT;
   private static final double footLength = 0.28;  //TODO update with foot sole length
   private static final double footWidth = 0.12;   //TODO update with foot sole width
   public static final Color defaultRightColor = new Color(0.15f, 0.8f, 0.15f, 1.0f);

   private final SideDependentList<FootSpoof> contactableFeet = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   private final SideDependentList<ReferenceFrame> soleZUpFrames = new SideDependentList<>();

   private final SideDependentList<FramePose3D> footPosesAtTouchdown = new SideDependentList<FramePose3D>(new FramePose3D(), new FramePose3D());
   private final SideDependentList<YoFramePoseUsingYawPitchRoll> currentFootPoses = new SideDependentList<>();

   private final SimulationConstructionSet scs;
   private final YoDouble yoTime;

   private final YoRegistry registry = new YoRegistry("ICPViz");

   private final YoFramePoint3D yoDesiredCoP = new YoFramePoint3D("desiredCoP", worldFrame, registry);
   private final YoFramePoseUsingYawPitchRoll yoNextFootstepPose = new YoFramePoseUsingYawPitchRoll("nextFootstepPose", worldFrame, registry);
   private final YoFrameConvexPolygon2D yoNextFootstepPolygon = new YoFrameConvexPolygon2D("nextFootstep", "", worldFrame, 4, registry);

   private final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

   private final int simulatedTicksPerGraphicUpdate = 1;

   private final Footstep footstep;
   private final ExoskeletonFootsole footsole;
   private final YoBoolean copPlanner = new YoBoolean("copPlanner", registry);

   public FootsoleCoPPlotter()
   {
      copPlanner.set(false);

      // create foot
      double xToAnkle = 0.0;
      double yToAnkle = 0.0;
      double zToAnkle = 0.0;
      List<Point2D> contactPointsInSoleFrame = new ArrayList<Point2D>();
      contactPointsInSoleFrame.add(new Point2D(footLength / 2.0, footWidth / 2.0));
      contactPointsInSoleFrame.add(new Point2D(footLength / 2.0, -footWidth / 2.0));
      contactPointsInSoleFrame.add(new Point2D(-footLength / 2.0, -footWidth / 2.0));
      contactPointsInSoleFrame.add(new Point2D(-footLength / 2.0, footWidth / 2.0));
      FootSpoof contactableFoot = new FootSpoof("Foot", xToAnkle, yToAnkle, zToAnkle, contactPointsInSoleFrame, 0.0);
      FramePose3D startingPose = footPosesAtTouchdown.get(robotSide);
      startingPose.setToZero(worldFrame);
      contactableFoot.setSoleFrame(startingPose);
      contactableFeet.put(robotSide, contactableFoot);

      currentFootPoses.put(robotSide, new YoFramePoseUsingYawPitchRoll("FootPose", worldFrame, registry));

      Graphics3DObject footGraphics = new Graphics3DObject();
      AppearanceDefinition footColor = YoAppearance.Color(defaultRightColor);
      footGraphics.addExtrudedPolygon(contactPointsInSoleFrame, 0.02, footColor);
      yoGraphicsListRegistry.registerYoGraphic("FootViz", new YoGraphicShape(robotSide.getCamelCaseName() + "FootViz", footGraphics, currentFootPoses.get(robotSide), 1.0));

      FootSpoof rightContactableFoot = contactableFeet.get(robotSide);

      soleZUpFrames.put(robotSide, new ZUpFrame(worldFrame, rightContactableFoot.getSoleFrame(), robotSide.getCamelCaseNameForStartOfExpression() + "ZUp"));
      soleFrames.put(robotSide, rightContactableFoot.getSoleFrame());

      footstep = new Footstep(robotSide, startingPose);
      footstep.setPredictedContactPoints(contactableFeet.get(robotSide).getContactPoints2d());

      footsole = new ExoskeletonFootsole(footstep.getSoleReferenceFrame());
      setupFootsole();

      updatables.add(new Updatable()
      {
         @Override
         public void update(double time)
         {
            soleFrames.get(robotSide).update();
            soleZUpFrames.get(robotSide).update();
         }
      });

      YoGraphicPosition desiredCoPGraphic = new YoGraphicPosition("desiredCoP", yoDesiredCoP, 0.01, YoAppearance.Black(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
      yoGraphicsListRegistry.registerYoGraphic("desiredCoP", desiredCoPGraphic);
      yoGraphicsListRegistry.registerArtifact("desiredCoP", desiredCoPGraphic.createArtifact());

      yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("nextFootstep", yoNextFootstepPolygon, Color.blue, false));

      int NUMBER_OF_PRESSURE_SPOTS = footsole.yoPressureSpaces.size();
      for(int i = 0; i < NUMBER_OF_PRESSURE_SPOTS; i++)
         yoGraphicsListRegistry.registerArtifact("upcomingFootsteps", new YoArtifactPolygon("pressurePoint"+i, footsole.getPressureSensor(i), Color.red, false));

      Graphics3DObject footstepGraphics = new Graphics3DObject();
      List<Point2D> contactPoints = new ArrayList<Point2D>();
      for (FramePoint2D point : contactableFeet.get(robotSide).getContactPoints2d())
         contactPoints.add(new Point2D(point));
      footstepGraphics.addExtrudedPolygon(contactPoints, 0.02, YoAppearance.Color(Color.blue));
      yoGraphicsListRegistry.registerYoGraphic("upcomingFootsteps", new YoGraphicShape("nextFootstep", footstepGraphics, yoNextFootstepPose, 1.0));

      SimulationConstructionSetParameters scsParameters = new SimulationConstructionSetParameters(true, BUFFER_SIZE);
      Robot robot = new Robot("Footsole");
      yoTime = robot.getYoTime();
      scs = new SimulationConstructionSet(robot, scsParameters);
      scs.addYoRegistry(registry);
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setPlaybackRealTimeRate(0.025);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCoordinateSystem(0.3);
      scs.addStaticLinkGraphics(linkGraphics);
      scs.setCameraFix(0.0, 0.0, 0.0);
      scs.setCameraPosition(-0.5, -0.5, 0.5);

      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
      simulationOverheadPlotterFactory.createOverheadPlotter();

      scs.startOnAThread();
      simulate();
      ThreadTools.sleepForever();
   }

   private final FrameConvexPolygon2D footstepPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D tempFootstepPolygonForShrinking = new FrameConvexPolygon2D();
   private final ConvexPolygonScaler convexPolygonShrinker = new ConvexPolygonScaler();
   private final FrameConvexPolygon2D testSensorPolygon = new FrameConvexPolygon2D();

   private void simulate()
   {

      callUpdatables();

      while (yoTime.getDoubleValue() < 30.0)
      {
         updateFootPressure(yoTime.getDoubleValue());

         simulateOneTick();
         if (copPlanner.getBooleanValue())
            break;
      }
   }

   private void updateFootPressure(double time)
   {
      if( time > 2.0 && time < 4.0){
         footsole.setCoP(0, 1.0);
      }
      else if(time > 4.0 && time < 6.0){
         footsole.setCoP(0, 0.0);
         footsole.setCoP(1, 1.0);
      }
      else if(time > 6.0 && time < 8.0){
         footsole.setCoP(1, 0.0);
         footsole.setCoP(2, 1.0);
      }
      else if(time > 8.0 && time < 10.0){
         footsole.setCoP(2, 0.0);
         footsole.setCoP(3, 1.0);
      }
      else if(time > 10.0 && time < 12.0){
         footsole.setCoP(3, 0.0);
         footsole.setCoP(4, 1.0);
      }
      else if(time > 12.0 && time < 14.0){
         footsole.setCoP(4, 0.0);
         footsole.setCoP(5, 1.0);
      }
      else if(time > 14.0 && time < 16.0){
         footsole.setCoP(5, 0.0);
         footsole.setCoP(6, 1.0);
      }
      else if(time > 16.0 && time < 18.0){
         footsole.setCoP(6, 0.0);
         footsole.setCoP(7, 1.0);
      }
      else if(time > 18.0 && time < 20.0){
         footsole.setCoP(7, 0.0);
      }
      else if(time > 20.0 && time < 22.0){
         footsole.setCoP(0, 1.0);
         footsole.setCoP(2, 1.0);
      }
      else if(time > 22.0 && time < 24.0){
         footsole.setCoP(4, 1.0);
         footsole.setCoP(6, 1.0);
      }
      else if(time > 24.0 && time < 26.0) {
         footsole.setCoP(0, 1.0);
         footsole.setCoP(1, 1.0);
         footsole.setCoP(2, 0.0);
         footsole.setCoP(4, 0.0);
         footsole.setCoP(6, 0.0);
      }
      else if(time > 26.0 && time < 28.0) {
         footsole.setCoP(0, 1.0);
         footsole.setCoP(1, 0.0);
         footsole.setCoP(7, 5.0);
      }

      footsole.updateCoP();
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

   }

   private final FramePoint3D desiredCoP = new FramePoint3D();
   private final FrameVector3D desiredCoPVelocity = new FrameVector3D();
   private final FrameVector3D desiredCoPAcceleration = new FrameVector3D();

   private void simulateOneTick()
   {
      callUpdatables();
      updateFootViz();

      yoDesiredCoP.set(desiredCoP);

      yoTime.add(dt);

      scs.tickAndUpdate();
   }

   private void updateFootViz()
   {
      FramePose3D footPose = new FramePose3D(contactableFeet.get(robotSide).getSoleFrame());
      footPose.changeFrame(worldFrame);
      currentFootPoses.get(robotSide).set(footPose);
   }

   private void callUpdatables()
   {
      for (Updatable updatable : updatables)
         updatable.update(yoTime.getDoubleValue());
   }

   public static void main(String[] args)
   {
      new FootsoleCoPPlotter();
   }

   public class ExoskeletonFootsole
   {
      private final List<Point2D> sensorPointsInSoleFrame = new ArrayList<Point2D>();
      private static final double sensorLength = 0.02;
      private static final double sensorWidth = 0.02;

      private final ReferenceFrame soleFrame;
      private final ArrayList<YoFrameConvexPolygon2D> yoPressureSpaces = new ArrayList<>();
      private final FrameConvexPolygon2D tmpSensorPolygon = new FrameConvexPolygon2D();

      private HashMap<Integer, YoDouble> pressureSensorValueMap = new HashMap<>();

      private final YoFramePoint2D yoMeasuredCoP = new YoFramePoint2D("measuredCoP", worldFrame, registry);

      public ExoskeletonFootsole(ReferenceFrame soleFrame)
      {
         this.soleFrame = soleFrame;
         initializePressureSensorShape();
         initializePressureSensorCentroids();

         YoGraphicPosition measuredCoPGraphic = new YoGraphicPosition("measuredCoP", yoMeasuredCoP, 0.01, YoAppearance.Red(), YoGraphicPosition.GraphicType.BALL_WITH_CROSS);
         yoGraphicsListRegistry.registerYoGraphic("desiredCoP", measuredCoPGraphic);
         yoGraphicsListRegistry.registerArtifact("desiredCoP", measuredCoPGraphic.createArtifact());
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
         addCentroid(0, 0.03656 , 0.02409);
         addCentroid(1, 0.09356 , 0.01633);
         addCentroid(2, 0.08634 , -0.03239);
         addCentroid(3, 0.03448 , -0.02511);
         addCentroid(4, -0.02652, -0.02389);
         addCentroid(5, -0.09118, -0.03123);
         addCentroid(6, -0.09421, 0.01821);
         addCentroid(7, -0.03452, 0.02498);

         for(int i = 0; i < yoPressureSpaces.size(); i++)
            pressureSensorValueMap.put(i, new YoDouble("pressureCoP"+i, registry));
      }

      private void addCentroid(int sensorNumber, double x, double y)
      {
         tmpSensorPolygon.setIncludingFrame(footstep.getSoleReferenceFrame(), Vertex2DSupplier.asVertex2DSupplier(sensorPointsInSoleFrame));
         tmpSensorPolygon.translate(x, y);
         tmpSensorPolygon.changeFrameAndProjectToXYPlane(worldFrame);
         yoPressureSpaces.add(new YoFrameConvexPolygon2D("pressurePoint"+sensorNumber, "", worldFrame, 4, registry));
         yoPressureSpaces.get(sensorNumber).set(tmpSensorPolygon);
      }

      public YoFrameConvexPolygon2D getPressureSensor(int sensorNumber)
      {
         return yoPressureSpaces.get(sensorNumber);
      }

      public void setCoP(int sensorNumber, double normalForce)
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
}
