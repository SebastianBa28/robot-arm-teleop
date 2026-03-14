//
//  Models.swift
//  robot_arm_teleop
//
//  Created by Gemini on 3/13/26.
//

import Foundation
import simd

/// Twist velocity data (internal, before remapping).
struct TwistData {
    let vx: Double
    let vy: Double
    let vz: Double
    let wx: Double
    let wy: Double
    let wz: Double
}

/// Full message sent to the bridge over WebSocket.
struct TeleopMessage: Encodable {
    let vx: Double
    let vy: Double
    let vz: Double
    let wx: Double
    let wy: Double
    let wz: Double
    let transform: [[Float]]  // 4x4 row-major
    let mode: String           // "velocity" or "position"
}

/// Feedback data received from the bridge.
struct FeedbackData: Decodable {
    let manipulability: Double
    let joint_limit_proximity: [Double]
    let workspace_proximity: Double
    let is_feasible: Bool
}

// MARK: - Axis Mapping

/// Which phone axis (and sign) maps to a robot axis.
enum AxisSource: String, CaseIterable, Identifiable {
    case posX = "+X"
    case negX = "-X"
    case posY = "+Y"
    case negY = "-Y"
    case posZ = "+Z"
    case negZ = "-Z"

    var id: String { rawValue }

    /// Extract the mapped component from a 3D vector.
    func apply(_ x: Double, _ y: Double, _ z: Double) -> Double {
        switch self {
        case .posX: return x
        case .negX: return -x
        case .posY: return y
        case .negY: return -y
        case .posZ: return z
        case .negZ: return -z
        }
    }

    /// Extract the mapped component from a Float 3D vector.
    func applyF(_ x: Float, _ y: Float, _ z: Float) -> Float {
        switch self {
        case .posX: return x
        case .negX: return -x
        case .posY: return y
        case .negY: return -y
        case .posZ: return z
        case .negZ: return -z
        }
    }
}

struct AxisMapping {
    var robotX: AxisSource  // which phone axis → robot X
    var robotY: AxisSource  // which phone axis → robot Y
    var robotZ: AxisSource  // which phone axis → robot Z

    static let `default` = AxisMapping(robotX: .negZ, robotY: .negX, robotZ: .posY)

    /// Remap a 3D vector (e.g., linear or angular velocity).
    func remap(_ x: Double, _ y: Double, _ z: Double) -> (Double, Double, Double) {
        return (robotX.apply(x, y, z), robotY.apply(x, y, z), robotZ.apply(x, y, z))
    }

    /// Remap a 4x4 transform matrix (rotation + translation).
    func remapTransform(_ m: simd_float4x4) -> [[Float]] {
        // Extract translation
        let tx = m.columns.3.x, ty = m.columns.3.y, tz = m.columns.3.z

        // Remap translation
        let newTx = robotX.applyF(tx, ty, tz)
        let newTy = robotY.applyF(tx, ty, tz)
        let newTz = robotZ.applyF(tx, ty, tz)

        // Remap rotation columns then rows
        // Each column of the rotation represents a basis vector that needs remapping
        var result = [[Float]](repeating: [Float](repeating: 0, count: 4), count: 4)

        for col in 0..<3 {
            let cx = m[col].x, cy = m[col].y, cz = m[col].z
            // Remap the rows (output axes) of this column
            result[0][col] = robotX.applyF(cx, cy, cz)
            result[1][col] = robotY.applyF(cx, cy, cz)
            result[2][col] = robotZ.applyF(cx, cy, cz)
        }

        // Remap the columns (input axes) by swapping columns according to mapping
        // Actually we need to also permute columns — let's build a proper rotation matrix
        // R_robot = P * R_arkit * P^T where P is the permutation+sign matrix

        // Build the 3x3 permutation matrix P
        func permRow(_ src: AxisSource) -> (Int, Float) {
            switch src {
            case .posX: return (0,  1.0)
            case .negX: return (0, -1.0)
            case .posY: return (1,  1.0)
            case .negY: return (1, -1.0)
            case .posZ: return (2,  1.0)
            case .negZ: return (2, -1.0)
            }
        }

        let (xi, xs) = permRow(robotX)
        let (yi, ys) = permRow(robotY)
        let (zi, zs) = permRow(robotZ)

        // P matrix rows
        var P = simd_float3x3(0)
        P[xi] = simd_float3(xs, 0, 0)  // column xi gets row 0 contribution
        P[yi] = P[yi] + simd_float3(0, ys, 0)
        P[zi] = P[zi] + simd_float3(0, 0, zs)

        // Actually, build P properly as: P[row][col]
        // P maps arkit coords to robot coords: v_robot = P * v_arkit
        var Pmat = simd_float3x3(0)
        // Row 0 of P (robot X): picks arkit axis specified by robotX
        // Row 1 of P (robot Y): picks arkit axis specified by robotY
        // Row 2 of P (robot Z): picks arkit axis specified by robotZ
        // simd_float3x3 is column-major, so we build columns
        // P[col][row]
        let sources = [robotX, robotY, robotZ]
        for row in 0..<3 {
            let (srcIdx, sign) = permRow(sources[row])
            Pmat[srcIdx][row] = sign
        }

        // R_arkit as simd_float3x3
        let Rarkit = simd_float3x3(
            simd_make_float3(m.columns.0),
            simd_make_float3(m.columns.1),
            simd_make_float3(m.columns.2)
        )

        let Rrobot = Pmat * Rarkit * Pmat.transpose

        // Build output 4x4 row-major
        var out = [[Float]](repeating: [Float](repeating: 0, count: 4), count: 4)
        for row in 0..<3 {
            for col in 0..<3 {
                out[row][col] = Rrobot[col][row]  // column-major to row-major
            }
        }
        out[0][3] = newTx
        out[1][3] = newTy
        out[2][3] = newTz
        out[3][3] = 1.0

        return out
    }
}
