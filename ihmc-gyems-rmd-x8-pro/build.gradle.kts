plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.7"
   id("us.ihmc.ihmc-cd") version "1.24"
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
   api("us.ihmc:ihmc-gyems-core:source")
}

testDependencies {
   api(ihmc.sourceSetProject("main"))
}
