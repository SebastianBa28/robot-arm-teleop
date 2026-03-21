//
//  TeleopManager.swift
//  robot_arm_teleop
//

import Combine
import simd

class TeleopManager: ObservableObject {
    let webSocketManager = WebSocketManager()
    let arSessionManager = ARSessionManager()
    let hapticsController: HapticsController

    @Published var isARActive: Bool = false

    // Configuration (set from ContentView @AppStorage bindings)
    var controlMode: String = "velocity"
    var axisMapping: AxisMapping = .default

    // Reference pose for position mode (captured when AR starts)
    private var referencePose: simd_float4x4?

    // Pass-through properties
    var isConnected: Bool { webSocketManager.isConnected }
    var isConnecting: Bool { webSocketManager.isConnecting }
    var currentPose: simd_float4x4 { arSessionManager.currentPose }
    var isTracking: Bool { arSessionManager.isTracking }
    var trackingError: String? { arSessionManager.trackingError }
    var feedback: FeedbackData? { webSocketManager.lastFeedback }
    var connectionError: String? { webSocketManager.connectionError }
    var isLiDARActive: Bool { arSessionManager.isLiDARActive }

    private var cancellables = Set<AnyCancellable>()

    init() {
        self.hapticsController = HapticsController(webSocketManager: webSocketManager)

        // Forward teleop data to WebSocket when streaming
        arSessionManager.$latestTwist
            .compactMap { $0 }
            .sink { [weak self] twist in
                guard let self = self, self.isARActive, self.webSocketManager.isConnected else { return }

                let mapping = self.axisMapping
                let mode = self.controlMode

                // Remap twist
                let (rvx, rvy, rvz) = mapping.remap(twist.vx, twist.vy, twist.vz)
                let (rwx, rwy, rwz) = mapping.remap(twist.wx, twist.wy, twist.wz)

                // Get current pose and compute relative transform
                let currentPose = self.arSessionManager.currentPose
                if self.referencePose == nil {
                    self.referencePose = currentPose
                }
                let relativePose = simd_mul(simd_inverse(self.referencePose!), currentPose)

                // Remap the relative transform
                let remappedTransform = mapping.remapTransform(relativePose)

                let message = TeleopMessage(
                    vx: rvx, vy: rvy, vz: rvz,
                    wx: rwx, wy: rwy, wz: rwz,
                    transform: remappedTransform,
                    mode: mode,
                    command: nil
                )
                self.webSocketManager.send(message: message)
            }
            .store(in: &cancellables)

        // Re-publish changes from sub-managers so SwiftUI picks them up
        webSocketManager.objectWillChange
            .sink { [weak self] _ in self?.objectWillChange.send() }
            .store(in: &cancellables)
        arSessionManager.objectWillChange
            .sink { [weak self] _ in self?.objectWillChange.send() }
            .store(in: &cancellables)
    }

    func toggleAR() {
        if isARActive {
            arSessionManager.stop()
            referencePose = nil
        } else {
            arSessionManager.start()
            referencePose = nil  // will be captured on first frame
        }
        isARActive.toggle()
    }

    func resetRobot() {
        webSocketManager.sendCommand("reset")
        referencePose = nil
    }

    func connect(url: String) {
        webSocketManager.connect(to: url)
    }

    func disconnect() {
        webSocketManager.disconnect()
    }
}
