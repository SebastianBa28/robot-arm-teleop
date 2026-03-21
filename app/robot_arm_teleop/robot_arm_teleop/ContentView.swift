//
//  ContentView.swift
//  robot_arm_teleop
//
//  Created by Firdavs Nasriddinov on 3/13/26.
//

import SwiftUI
import simd

struct ContentView: View {
    @EnvironmentObject var teleop: TeleopManager

    @AppStorage("serverURL") private var serverURL: String = "ws://192.168.1.100:8000/ws"
    @AppStorage("controlMode") private var controlMode: String = "velocity"
    @AppStorage("axisMapX") private var axisMapX: String = "-Z"
    @AppStorage("axisMapY") private var axisMapY: String = "-X"
    @AppStorage("axisMapZ") private var axisMapZ: String = "+Y"
    @AppStorage("showFeedback") private var showFeedback: Bool = true
    @State private var showSettings = false

    var body: some View {
        ZStack {
            Color(UIColor.systemBackground).edgesIgnoringSafeArea(.all)

            VStack {
                // Header
                HStack {
                    VStack(alignment: .leading, spacing: 2) {
                        Text("Robot Teleop")
                            .font(.headline)
                        Text(teleop.isLiDARActive ? "ARKit (LiDAR)" : "ARKit (Camera)")
                            .font(.caption2)
                            .foregroundColor(.secondary)
                    }
                    Spacer()

                    // Tracking status
                    if let error = teleop.trackingError {
                        Text(error)
                            .font(.caption2)
                            .foregroundColor(.orange)
                    }

                    // Connection status
                    if let error = teleop.connectionError {
                        Text(error)
                            .font(.caption2)
                            .foregroundColor(.red)
                            .lineLimit(1)
                    }

                    Circle()
                        .fill(teleop.isConnected ? Color.green : Color.red)
                        .frame(width: 10, height: 10)

                    Button(action: { showSettings = true }) {
                        Image(systemName: "gearshape.fill")
                            .font(.title2)
                            .padding(.leading, 10)
                    }
                }
                .padding()

                Spacer()

                // Pose Display
                VStack(spacing: 20) {
                    Text("Device Pose (Camera Transform) — meters")
                        .font(.subheadline)
                        .foregroundColor(.secondary)

                    PoseMatrixView(matrix: teleop.currentPose)
                        .padding()
                        .background(Color(UIColor.secondarySystemBackground))
                        .cornerRadius(12)
                        .shadow(radius: 5)

                    if showFeedback, let feedback = teleop.feedback {
                        FeedbackView(feedback: feedback)
                    }
                }

                Spacer()

                // Control Buttons
                HStack(spacing: 30) {
                    // Reset Button
                    Button(action: { resetRobot() }) {
                        ZStack {
                            Circle()
                                .fill(Color.blue)
                                .frame(width: 60, height: 60)
                                .shadow(radius: 8)

                            Image(systemName: "arrow.counterclockwise")
                                .font(.title2)
                                .foregroundColor(.white)
                        }
                    }

                    // AR Toggle Button
                    Button(action: { toggleAR() }) {
                        ZStack {
                            Circle()
                                .fill(teleop.isARActive ? Color.green : Color.red)
                                .frame(width: 80, height: 80)
                                .shadow(radius: 10)

                            Image(systemName: teleop.isARActive ? "pause.fill" : "play.fill")
                                .font(.title)
                                .foregroundColor(.white)
                        }
                    }
                }
                .padding(.bottom, 40)
            }
        }
        .sheet(isPresented: $showSettings) {
            SettingsView(
                serverURL: $serverURL,
                controlMode: $controlMode,
                axisMapX: $axisMapX,
                axisMapY: $axisMapY,
                axisMapZ: $axisMapZ,
                showFeedback: $showFeedback,
                isPresented: $showSettings,
                connectAction: { teleop.connect(url: serverURL) },
                disconnectAction: { teleop.disconnect() },
                isConnected: teleop.isConnected,
                isConnecting: teleop.isConnecting,
                connectionError: teleop.connectionError
            )
        }
        .onChange(of: controlMode) { newValue in
            teleop.controlMode = newValue
        }
        .onChange(of: axisMapX) { _ in syncAxisMapping() }
        .onChange(of: axisMapY) { _ in syncAxisMapping() }
        .onChange(of: axisMapZ) { _ in syncAxisMapping() }
        .onAppear { syncAxisMapping(); teleop.controlMode = controlMode }
    }

    private func syncAxisMapping() {
        teleop.axisMapping = AxisMapping(
            robotX: AxisSource(rawValue: axisMapX) ?? .negZ,
            robotY: AxisSource(rawValue: axisMapY) ?? .negX,
            robotZ: AxisSource(rawValue: axisMapZ) ?? .posY
        )
    }

    private func resetRobot() {
        teleop.resetRobot()
        let impact = UIImpactFeedbackGenerator(style: .heavy)
        impact.impactOccurred()
    }

    private func toggleAR() {
        teleop.toggleAR()
        let impact = UIImpactFeedbackGenerator(style: .medium)
        impact.impactOccurred()
    }
}

// Helper view to display 4x4 matrix
struct PoseMatrixView: View {
    let matrix: simd_float4x4

    var body: some View {
        VStack(spacing: 8) {
            ForEach(0..<4) { row in
                HStack(spacing: 12) {
                    Text(String(format: "%.2f", matrix[0][row]))
                    Text(String(format: "%.2f", matrix[1][row]))
                    Text(String(format: "%.2f", matrix[2][row]))
                    Text(String(format: "%.2f", matrix[3][row]))
                }
                .font(.system(.body, design: .monospaced))
            }
        }
    }
}

// Helper view for feedback
struct FeedbackView: View {
    let feedback: FeedbackData

    var body: some View {
        HStack(spacing: 20) {
            VStack {
                Text("Manipulability")
                    .font(.caption)
                Text(String(format: "%.3f", feedback.manipulability))
                    .foregroundColor(feedback.manipulability < 0.05 ? .red : .primary)
                    .bold()
            }

            VStack {
                Text("Feasible")
                    .font(.caption)
                Image(systemName: feedback.is_feasible ? "checkmark.circle.fill" : "xmark.circle.fill")
                    .foregroundColor(feedback.is_feasible ? .green : .red)
            }
        }
        .padding()
        .background(Color(UIColor.tertiarySystemBackground))
        .cornerRadius(10)
    }
}

#Preview {
    ContentView()
        .environmentObject(TeleopManager())
}
