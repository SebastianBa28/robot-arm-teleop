//
//  robot_arm_teleopApp.swift
//  robot_arm_teleop
//

import SwiftUI

@main
struct robot_arm_teleopApp: App {
    @StateObject private var teleopManager = TeleopManager()

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(teleopManager)
        }
    }
}
