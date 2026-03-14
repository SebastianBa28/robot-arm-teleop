//
//  Models.swift
//  robot_arm_teleop
//
//  Created by Gemini on 3/13/26.
//

import Foundation
import simd

/// Twist velocity data to be sent to the bridge.
/// Format: [vx, vy, vz, wx, wy, wz]
struct TwistData: Encodable {
    let vx: Double
    let vy: Double
    let vz: Double
    let wx: Double
    let wy: Double
    let wz: Double

    func toArray() -> [Double] {
        return [vx, vy, vz, wx, wy, wz]
    }
}

/// Feedback data received from the bridge.
struct FeedbackData: Decodable {
    let manipulability: Double
    let joint_limit_proximity: [Double]
    let workspace_proximity: Double
    let is_feasible: Bool
}
