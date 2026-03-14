//
//  HapticsController.swift
//  robot_arm_teleop
//

import CoreHaptics
import Combine

class HapticsController: ObservableObject {
    private var engine: CHHapticEngine?
    private var cancellables = Set<AnyCancellable>()
    private var singularityPlayer: CHHapticAdvancedPatternPlayer?
    private var supportsHaptics: Bool

    init(webSocketManager: WebSocketManager) {
        self.supportsHaptics = CHHapticEngine.capabilitiesForHardware().supportsHaptics

        if supportsHaptics {
            prepareHaptics()
        }

        webSocketManager.$lastFeedback
            .receive(on: DispatchQueue.main)
            .sink { [weak self] feedback in
                self?.updateHaptics(feedback: feedback)
            }
            .store(in: &cancellables)
    }

    private func prepareHaptics() {
        do {
            engine = try CHHapticEngine()
            try engine?.start()

            let intensity = CHHapticEventParameter(parameterID: .hapticIntensity, value: 0)
            let sharpness = CHHapticEventParameter(parameterID: .hapticSharpness, value: 0.5)
            let event = CHHapticEvent(eventType: .hapticContinuous, parameters: [intensity, sharpness], relativeTime: 0, duration: 100)
            let pattern = try CHHapticPattern(events: [event], parameters: [])
            singularityPlayer = try engine?.makeAdvancedPlayer(with: pattern)
        } catch {
            print("Haptics setup error: \(error.localizedDescription)")
            engine = nil
            singularityPlayer = nil
        }
    }

    private func updateHaptics(feedback: FeedbackData?) {
        guard supportsHaptics, let player = singularityPlayer else { return }

        guard let feedback = feedback else {
            try? player.stop(atTime: CHHapticTimeImmediate)
            return
        }

        // Map low manipulability to high intensity
        let threshold: Double = 0.2
        var intensity: Float = 0.0

        if feedback.manipulability < threshold {
            intensity = Float(1.0 - (feedback.manipulability / threshold))
        }

        // Joint limits: if any joint > 0.9 proximity, add intensity
        let maxLimit = feedback.joint_limit_proximity.max() ?? 0.0
        if maxLimit > 0.9 {
            intensity = max(intensity, Float((maxLimit - 0.9) * 10.0))
        }

        intensity = min(max(intensity, 0.0), 1.0)

        if intensity > 0.05 {
            let param = CHHapticDynamicParameter(parameterID: .hapticIntensityControl, value: intensity, relativeTime: 0)
            do {
                try player.start(atTime: CHHapticTimeImmediate)
                try player.sendParameters([param], atTime: CHHapticTimeImmediate)
            } catch {
                print("Haptics update error: \(error)")
            }
        } else {
            try? player.stop(atTime: CHHapticTimeImmediate)
        }
    }
}
