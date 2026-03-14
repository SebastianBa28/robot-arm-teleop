//
//  robot_arm_teleopApp.swift
//  robot_arm_teleop
//
//  Created by Firdavs Nasriddinov on 3/13/26.
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
