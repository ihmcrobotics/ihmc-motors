package us.ihmc.sensors.LoadStarILoad;

import javafx.animation.AnimationTimer;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.geometry.Pos;
import javafx.scene.Scene;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.chart.CategoryAxis;
import javafx.scene.chart.LineChart;
import javafx.scene.chart.NumberAxis;
import javafx.scene.chart.XYChart;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.StackPane;
import javafx.scene.layout.VBox;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.scene.text.FontWeight;
import javafx.stage.Stage;

import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class LoadStarILoadUI extends Application
{
   private final String COM_PORT = "COM3";
   private Stage stage;
   private StackPane root;
   private LoadStarILoad load;
   private double weightN;
   private double weightLb;
   private final int WINDOW_SIZE = 100;
   private ScheduledExecutorService scheduledExecutorService;

   public static void main(String[] args) throws IOException
   {
      launch(args);
   }

   @Override
   public void start(Stage primaryStage) throws IOException
   {
      initializeSerialIfNull();

      stage = primaryStage;
      stage.setTitle("Loadstar Sensors");
      root = new StackPane();


      Label labelInit = new Label("Connected to " + COM_PORT);

      Button buttonStart = new Button();
      buttonStart.setText("Run");
      buttonStart.setOnAction(new EventHandler<ActionEvent>() {
         @Override
         public void handle(ActionEvent event) {
            graphWeight();
         }
      });

      VBox vbox = new VBox(5);
      vbox.setAlignment(Pos.CENTER);
      vbox.getChildren().addAll(labelInit, buttonStart);

      root.getChildren().add(vbox);

      Scene scene = new Scene(root, 350, 250);
      stage.setScene(scene);

      stage.show();
   }

   private void initializeSerialIfNull() throws IOException
   {
      if(load == null)
      {
         load = new LoadStarILoad(COM_PORT);
         load.tare();
      }
   }

   public void outputWeightScene()
   {
      //root = new StackPane();

      load.outputWeightOnce();
      weightN = load.getForceNewton();
      weightLb = load.getForcePound();
      Label labelWeight =  new Label("Weight: ");
      Label labelNewtons = new Label("Newtons: " + weightN);
      Label labelPounds =  new Label("Pounds:  " + weightLb);

      Button buttonTare = new Button();
      buttonTare.setText("Tare");
      buttonTare.setOnAction(new EventHandler<ActionEvent>() {

         @Override
         public void handle(ActionEvent event) {
            load.tare();
            outputWeightScene();
         }
      });

      Button buttonRefresh = new Button();
      buttonRefresh.setText("Refresh");
      buttonRefresh.setOnAction(new EventHandler<ActionEvent>() {

         @Override
         public void handle(ActionEvent event) {
            outputWeightScene();
         }
      });

      GridPane grid = new GridPane();
      grid.setAlignment(Pos.CENTER);
      grid.setHgap(16);
      grid.add(labelWeight, 0, 0);
      grid.add(labelNewtons, 1, 0);
      grid.add(labelPounds, 1, 1);
      grid.add(buttonRefresh, 1, 3);
      grid.add(buttonTare, 1, 4);

      root.getChildren().add(grid);
      updateScene(root);

      sleep(100);
      outputWeightScene();
   }

   public void testScene() {
      Canvas canvas = new Canvas(350, 250);

      root.getChildren().add(canvas);

      GraphicsContext gc = canvas.getGraphicsContext2D();
      gc.setFont(Font.font("Helvetica", FontWeight.BOLD, 24));
      gc.setStroke(Color.BLACK);
      gc.setLineWidth(1);

      new AnimationTimer()
      {
         public void handle(long currentNanoTime)
         {
            gc.setFill(Color.WHITE);
            gc.fillRect(0,0,350,250);
            load.outputWeightOnce();

            String pointsText = "Weight: " + load.getForceNewton();
            gc.fillText(pointsText, 10, 250/2);
            gc.strokeText(pointsText, 10, 250/2);
         }
      }.start();

      stage.show();
   }

   public void graphWeight() {
      //defining the axes
      final CategoryAxis xAxis = new CategoryAxis(); // we are gonna plot against time
      final NumberAxis yAxis = new NumberAxis();
      xAxis.setLabel("Time (ms)");
      xAxis.setAnimated(false); // axis animations are removed
      yAxis.setLabel("Weight (lb)");
      yAxis.setAnimated(false); // axis animations are removed

      //creating the line chart with two axis created above
      final LineChart<String, Number> lineChart = new LineChart<>(xAxis, yAxis);
      lineChart.setTitle("iLoad Pro Weight Output");
      lineChart.setAnimated(false); // disable animations
      lineChart.setPrefSize(1200, 1500);
      lineChart.setMinSize(1000, 800);
      lineChart.setMaxSize(6000, 6000);

      //defining a series to display data
      XYChart.Series<String, Number> series = new XYChart.Series<>();
      series.setName("Data Series");

      // add series to chart
      lineChart.getData().add(series);

      Button buttonStop = new Button();
      buttonStop.setText("Stop");
      buttonStop.setOnAction(new EventHandler<ActionEvent>() {
         @Override
         public void handle(ActionEvent event) {
            try
            {
               stop();
            }
            catch (Exception e) {}
         }
      });

      Button buttonRestart = new Button();
      buttonRestart.setText("Restart");
      buttonRestart.setOnAction(new EventHandler<ActionEvent>() {
         @Override
         public void handle(ActionEvent event) {
            try
            {
               start(stage);
            }
            catch (Exception e) {}
         }
      });

      Button buttonTare = new Button();
      buttonTare.setText("Tare");
      buttonTare.setOnAction(new EventHandler<ActionEvent>() {
         @Override
         public void handle(ActionEvent event) {
            try
            {
               //stop();
               //load.tare();
               //graphWeight();
            }
            catch (Exception e) {}
         }
      });

      HBox hbox = new HBox(5);
      hbox.setAlignment(Pos.CENTER);
      hbox.getChildren().addAll(buttonTare, buttonStop, buttonRestart);

      VBox vbox = new VBox(20);
      vbox.setAlignment(Pos.BOTTOM_CENTER);
      vbox.getChildren().addAll(lineChart, hbox);

      // setup scene
      Scene scene = new Scene(vbox, 1300, 1000);
      stage.setScene(scene);

      // this is used to display time in HH:mm:ss format
      final SimpleDateFormat simpleDateFormat = new SimpleDateFormat("mm:ss:SS");

      // setup a scheduled executor to periodically put data into the chart
      scheduledExecutorService = Executors.newSingleThreadScheduledExecutor();

      // put dummy data onto graph per second
      scheduledExecutorService.scheduleAtFixedRate(() -> {
         load.outputWeightOnce();
         Double weight = load.getForcePound();

         // Update the chart
         Platform.runLater(() -> {
            // get current time
            Date now = new Date();
            // put random number with current time
            series.getData().add(new XYChart.Data<>(simpleDateFormat.format(now), weight));

            if (series.getData().size() > WINDOW_SIZE)
               series.getData().remove(0);
         });
      }, 0, 1, TimeUnit.MILLISECONDS);
   }

   public void updateScene(StackPane root) {
      stage.setScene(new Scene(root, 300, 250));
      stage.show();
   }

   @Override
   public void stop() throws Exception {
      super.stop();
      scheduledExecutorService.shutdownNow();
   }

   private void sleep(long millis)
   {
      try
      {
         Thread.sleep(millis);
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }
   }
}
