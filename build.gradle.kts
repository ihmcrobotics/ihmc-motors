plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
   id("us.ihmc.ihmc-cd") version "1.26"
}

ihmc {
   group = "us.ihmc"
   version = "1.0.1"
   vcsUrl = "https://github.com/ihmcrobotics/ihmc-motors"
   openSource = true

   configureDependencyResolution()
   configurePublications()
}

val ihmcOpenRoboticsSoftwareVersion = "0.14.0-240126"

mainDependencies {
   api("us.ihmc:ihmc-ethercat-master:0.15.0")
   api("us.ihmc:ihmc-realtime:1.6.0")
   api("org.scream3r:jssc:2.8.0")
   api("us.ihmc:ihmc-robot-data-visualizer:$ihmcOpenRoboticsSoftwareVersion")
   api("us.ihmc:ihmc-robotics-toolkit:$ihmcOpenRoboticsSoftwareVersion")
}

testDependencies {
   api(ihmc.sourceSetProject("main"))
   api("us.ihmc:ihmc-commons-testing:0.32.0")
}

tasks.getByPath("installDist").dependsOn("compositeJar")

app.entrypoint("GyemsMotorTestBed", "us.ihmc.gyemsCore.teststands.GyemsMotorTestBed")
app.entrypoint("FootsoleTestBed", "us.ihmc.teststands.FootsoleTestBed")
app.entrypoint("TMotorKtTestBed", "us.ihmc.teststands.TMotorKtTestBed")
app.entrypoint("TMotorTestBed", "us.ihmc.teststands.TMotorTestBed")
