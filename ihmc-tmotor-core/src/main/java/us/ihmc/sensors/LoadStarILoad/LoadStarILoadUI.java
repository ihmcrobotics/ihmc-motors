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
import jssc.SerialPortException;

import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class LoadStarILoadUI extends Application
{
   private final String COM_PORT = "COM9";
   private Stage stage;
   private StackPane root;
   private LoadStarILoad load;
   private final int WINDOW_SIZE = 100;
   private ScheduledExecutorService scheduledExecutorService;

   public static void main(String[] args) throws IOException
   {
      launch(args);
   }

   @Override
   public void start(Stage primaryStage) throws IOException, SerialPortException {
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

   private void initializeSerialIfNull() throws IOException, SerialPortException {
      if(load == null)
      {
         load = new LoadStarILoad(COM_PORT);
      }
   }

   public void graphWeight() {
      final CategoryAxis xAxis = new CategoryAxis();
      final NumberAxis yAxis = new NumberAxis();
      xAxis.setLabel("Time (ms)");
      xAxis.setAnimated(false);
      yAxis.setLabel("Weight (N)");
      yAxis.setAnimated(false);

      final LineChart<String, Number> lineChart = new LineChart<>(xAxis, yAxis);
      lineChart.setTitle("iLoad Pro Weight Output");
      lineChart.setAnimated(false);
      lineChart.setPrefSize(1200, 1500);
      lineChart.setMinSize(1000, 800);
      lineChart.setMaxSize(6000, 6000);

      XYChart.Series<String, Number> series = new XYChart.Series<>();
      series.setName("Data Series");
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
               stop();
               load.tare();
               graphWeight();
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

      Scene scene = new Scene(vbox, 1300, 1000);
      stage.setScene(scene);

      final SimpleDateFormat simpleDateFormat = new SimpleDateFormat("mm:ss:SS");

      scheduledExecutorService = Executors.newSingleThreadScheduledExecutor();
      scheduledExecutorService.scheduleAtFixedRate(() -> {
         load.outputWeightOnce();
         Double weight = load.getForcePound();

         Platform.runLater(() -> {
            Date now = new Date();
            series.getData().add(new XYChart.Data<>(simpleDateFormat.format(now), weight));

            if (series.getData().size() > WINDOW_SIZE)
               series.getData().remove(0);
         });
      }, 0, 1, TimeUnit.MILLISECONDS);
   }

   @Override
   public void stop() throws Exception {
      super.stop();
      scheduledExecutorService.shutdownNow();
   }
}
