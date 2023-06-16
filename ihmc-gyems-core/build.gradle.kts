plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "8.3"
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
   api("us.ihmc:ihmc-motors-core:source")
}

testDependencies {
   api(ihmc.sourceSetProject("main"))
}

tasks.getByPath("installDist").dependsOn("compositeJar")

app.entrypoint("GyemsMotorTestBed", "us.ihmc.gyemsCore.teststands.GyemsMotorTestBed")

tasks.create("deploy") {
   dependsOn("installDist")

   doLast {
      remote.session("172.16.66.55", { sshClient -> sshClient.authPassword("root", "ShadyLady") }) //Note: Use 172.16.66.55 or 50 for ISA test beds. Use 100 for Electric actuator test beds. Later we should make this be a variable of sorts...
      {
         put(file("build/install/${project.name}").toString(), "isa-testbed")
         exec("chmod +x isa-testbed/bin/GyemsMotorTestBed")
      }
   }
}
