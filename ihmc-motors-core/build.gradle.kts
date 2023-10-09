plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
   id("us.ihmc.ihmc-cd") version "1.26"
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
   api("us.ihmc:ihmc-realtime:1.6.0")
   api("us.ihmc:ihmc-ethercat-master:0.14.0")
   api("us.ihmc:ihmc-robot-data-visualizer:source")
   api("us.ihmc:ihmc-robotics-toolkit:source")
}

testDependencies {
   api(ihmc.sourceSetProject("main"))
}