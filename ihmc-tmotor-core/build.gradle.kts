plugins {
   id("us.ihmc.ihmc-build")
   id("us.ihmc.ihmc-ci") version "7.4"
   id("us.ihmc.ihmc-cd") version "1.20"
   id("us.ihmc.log-tools-plugin") version "0.6.1"
   id("org.hidetake.ssh") version "2.1.1"
}

ihmc {
   loadProductProperties("../product.properties")

   configureDependencyResolution()
   repository("https://artifactory.ihmc.us/artifactory/proprietary-releases/")//, artifactoryUsername, artifactoryPassword)
   configurePublications()
}

mainDependencies {
   api("us.ihmc:ihmc-motors-core:source")
   api("us.ihmc:ihmc-ethercat-master:0.12.0")
   api("us.ihmc:ihmc-realtime:1.3.1")
   api("us.ihmc:eva:source")
   api("org.scream3r:jssc:2.8.0")
}

testDependencies {
   api(ihmc.sourceSetProject("main"))
}

tasks.getByPath("installDist").dependsOn("compositeJar")

app.entrypoint("TMotorKtTestBed", "us.ihmc.teststands.TMotorKtTestBed")
app.entrypoint("TMotorTestBed", "us.ihmc.teststands.TMotorTestBed")
//app.entrypoint("HipAndKneeControllerTestBed", "us.ihmc.teststands.HipAndKneeControllerTestBed")

tasks.create("deploy") {
   dependsOn("installDist")

   doLast {
      remote.session("172.16.66.55", { sshClient -> sshClient.authPassword("root", "ShadyLady") }) //Note: Use 172.16.66.55 or 50 for ISA test beds. Use 100 for Electric actuator test beds. Later we should make this be a variable of sorts...
              {
                 put(file("build/install/${project.name}").toString(), "isa-testbed")
                 exec("chmod +x isa-testbed/bin/TMotorKtTestBed")
                 exec("chmod +x isa-testbed/bin/TMotorTestBed")
//                 exec("chmod +x isa-testbed/bin/HipAndKneeControllerTestBed")
              }
   }
}