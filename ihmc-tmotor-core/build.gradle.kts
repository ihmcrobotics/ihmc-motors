plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.6"
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
   api("us.ihmc:ihmc-motors-core:source")
   api("us.ihmc:ihmc-ethercat-master:0.12.0")
   api("us.ihmc:ihmc-realtime:1.4.0")
   api("org.scream3r:jssc:2.8.0")
}

testDependencies {
   api(ihmc.sourceSetProject("main"))
}

tasks.getByPath("installDist").dependsOn("compositeJar")

app.entrypoint("FootsoleTestBed", "us.ihmc.teststands.FootsoleTestBed")
app.entrypoint("TMotorKtTestBed", "us.ihmc.teststands.TMotorKtTestBed")
app.entrypoint("TMotorTestBed", "us.ihmc.teststands.TMotorTestBed")

tasks.create("deploy") {
   dependsOn("installDist")

   doLast {
      remote.session("172.16.66.55", { sshClient -> sshClient.authPassword("root", "ShadyLady") }) //Note: Use 172.16.66.55 or 50 for ISA test beds. Use 100 for Electric actuator test beds. Later we should make this be a variable of sorts...
      {
         put(file("build/install/${project.name}").toString(), "isa-testbed")
         exec("chmod +x isa-testbed/bin/TMotorKtTestBed")
         exec("chmod +x isa-testbed/bin/TMotorTestBed")
         exec("chmod +x isa-testbed/bin/FootsoleTestBed")
      }
   }
}