plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.4"
   id("us.ihmc.ihmc-cd") version "1.21"
   id("us.ihmc.log-tools-plugin") version "0.6.2"
}

val artifactoryUsername: String by project
val artifactoryPassword: String by project

ihmc {
   loadProductProperties("../product.properties")

   configureDependencyResolution()
   repository("https://artifactory.ihmc.us/artifactory/proprietary-releases/", artifactoryUsername, artifactoryPassword)
   configurePublications()
}

mainDependencies {
   api("us.ihmc:ihmc-realtime:1.4.0")
   api("us.ihmc:ihmc-ethercat-master:0.12.0")
   api("us.ihmc:ihmc-robot-data-logger:0.20.9")
   api("us.ihmc:ihmc-robot-data-visualizer:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-common-walking-control-modules:source")   // TODO move to sensors build.gradle file
}

testDependencies {
   api(ihmc.sourceSetProject("main"))
}