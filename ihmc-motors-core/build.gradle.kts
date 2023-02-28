plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.7"
   id("us.ihmc.ihmc-cd") version "1.23"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
}

val nexusUsername: String by project
val nexusPassword: String by project

ihmc {
   loadProductProperties("../product.properties")

   configureDependencyResolution()
   repository("https://nexus.ihmc.us/repository/proprietary-releases/", nexusUsername, nexusPassword)
   configurePublications()
}

mainDependencies {
   api("us.ihmc:ihmc-realtime:1.5.1")
   api("us.ihmc:ihmc-ethercat-master:0.12.0")
   api("us.ihmc:ihmc-robot-data-visualizer:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")
   api("us.ihmc:ihmc-common-walking-control-modules:source")   // TODO move to sensors build.gradle file
}

testDependencies {
   api(ihmc.sourceSetProject("main"))
}