plugins {
    `java-library`
    kotlin("jvm") version "2.4.21"
}

group = "org.walledlakerobotics"
version = "2026.0.0"

repositories {
    mavenCentral()

    maven {
        name = "WPILib Release"
        url = uri("https://frcmaven.wpi.edu/artifactory/release/")
    }
}

dependencies {
    api("edu.wpi.first.wpilibj:wpilibj-java:2026.2.2")
    api("edu.wpi.first.wpiunits:wpiunits-java:2026.2.2")
    api("edu.wpi.first.wpimath:wpimath-java:2026.2.2")
    api("edu.wpi.first.wpiutil:wpiutil-java:2026.2.2")

    implementation("org.lz4:lz4-pure-java:1.8.0")
    implementation("com.fazecast:jSerialComm:[2.0.0,3.0.0)")
    implementation("org.jetbrains.kotlinx:kotlinx-io-core:0.9.1")
}

kotlin {
    jvmToolchain(17)
}

tasks.test {
    useJUnitPlatform()
}