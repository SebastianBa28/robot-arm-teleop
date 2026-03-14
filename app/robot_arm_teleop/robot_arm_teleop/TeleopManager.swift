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

    // Pass-through properties
    var isConnected: Bool { webSocketManager.isConnected }
    var currentPose: simd_float4x4 { arSessionManager.currentPose }
    var isTracking: Bool { arSessionManager.isTracking }
    var trackingError: String? { arSessionManager.trackingError }
    var feedback: FeedbackData? { webSocketManager.lastFeedback }
    var connectionError: String? { webSocketManager.connectionError }
    var isLiDARActive: Bool { arSessionManager.isLiDARActive }

    private var cancellables = Set<AnyCancellable>()

    init() {
        self.hapticsController = HapticsController(webSocketManager: webSocketManager)

        // Forward twist data to WebSocket when streaming
        arSessionManager.$latestTwist
            .compactMap { $0 }
            .sink { [weak self] twist in
                guard let self = self, self.isARActive, self.webSocketManager.isConnected else { return }
                self.webSocketManager.send(twist: twist)
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
        } else {
            arSessionManager.start()
        }
        isARActive.toggle()
    }

    func connect(url: String) {
        webSocketManager.connect(to: url)
    }

    func disconnect() {
        webSocketManager.disconnect()
    }
}
