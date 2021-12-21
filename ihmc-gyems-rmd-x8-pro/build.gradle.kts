plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.5"
   id("us.ihmc.ihmc-cd") version "1.22"
   id("us.ihmc.log-tools-plugin") version "0.6.3"
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
   api("us.ihmc:ihmc-gyems-core:source")
}

testDependencies {
   api(ihmc.sourceSetProject("main"))
}
