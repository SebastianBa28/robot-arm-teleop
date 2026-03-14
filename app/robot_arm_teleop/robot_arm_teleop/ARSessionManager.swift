//
//  ARSessionManager.swift
//  robot_arm_teleop
//

import ARKit
import Combine

class ARSessionManager: NSObject, ARSessionDelegate, ObservableObject {
    @Published var currentPose: simd_float4x4 = matrix_identity_float4x4
    @Published var isTracking: Bool = false
    @Published var latestTwist: TwistData?
    @Published var trackingError: String?
    @Published var isLiDARActive: Bool = false

    let session = ARSession()
    private let lock = NSLock()
    private var lastFrameTime: TimeInterval = 0
    private var lastTransform: simd_float4x4?

    override init() {
        super.init()
        session.delegate = self
    }

    func start() {
        guard ARWorldTrackingConfiguration.isSupported else {
            trackingError = "ARWorldTracking not supported on this device"
            return
        }
        let configuration = ARWorldTrackingConfiguration()
        configuration.worldAlignment = .gravity
        configuration.planeDetection = []
        if type(of: configuration).supportsFrameSemantics(.sceneDepth) {
            configuration.frameSemantics.insert(.sceneDepth)
            isLiDARActive = true
        } else {
            isLiDARActive = false
        }
        session.run(configuration, options: [.resetTracking, .removeExistingAnchors])
    }

    func stop() {
        session.pause()
        DispatchQueue.main.async {
            self.isTracking = false
        }
    }

    // MARK: - ARSessionDelegate

    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        let timestamp = frame.timestamp
        let currentTransform = frame.camera.transform

        lock.lock()
        let prevTransform = lastTransform
        let prevTime = lastFrameTime
        lastTransform = currentTransform
        lastFrameTime = timestamp
        lock.unlock()

        DispatchQueue.main.async {
            self.currentPose = currentTransform
        }

        guard let prevTransform = prevTransform, prevTime > 0 else { return }

        let dt = timestamp - prevTime
        if dt < 0.001 { return }

        // Linear velocity (world frame)
        let pCurr = currentTransform.columns.3
        let pPrev = prevTransform.columns.3
        let vx = Double(pCurr.x - pPrev.x) / dt
        let vy = Double(pCurr.y - pPrev.y) / dt
        let vz = Double(pCurr.z - pPrev.z) / dt

        // Angular velocity via relative rotation -> axis-angle
        let rCurr = simd_float3x3(
            simd_make_float3(currentTransform.columns.0),
            simd_make_float3(currentTransform.columns.1),
            simd_make_float3(currentTransform.columns.2)
        )
        let rPrev = simd_float3x3(
            simd_make_float3(prevTransform.columns.0),
            simd_make_float3(prevTransform.columns.1),
            simd_make_float3(prevTransform.columns.2)
        )
        let rRel = rCurr * rPrev.transpose
        let quat = simd_quatf(rRel)
        let angle = Double(quat.angle)
        let axis = quat.axis
        let angularSpeed = angle / dt

        let wx = Double(axis.x) * angularSpeed
        let wy = Double(axis.y) * angularSpeed
        let wz = Double(axis.z) * angularSpeed

        guard !vx.isNaN && !vy.isNaN && !vz.isNaN &&
              !wx.isNaN && !wy.isNaN && !wz.isNaN else { return }

        let twist = TwistData(vx: vx, vy: vy, vz: vz, wx: wx, wy: wy, wz: wz)
        DispatchQueue.main.async {
            self.latestTwist = twist
        }
    }

    func session(_ session: ARSession, cameraDidChangeTrackingState camera: ARCamera) {
        DispatchQueue.main.async {
            switch camera.trackingState {
            case .normal:
                self.isTracking = true
                self.trackingError = nil
            case .notAvailable:
                self.isTracking = false
                self.trackingError = "Tracking not available"
            case .limited(let reason):
                self.isTracking = true
                switch reason {
                case .excessiveMotion:
                    self.trackingError = "Too much motion"
                case .insufficientFeatures:
                    self.trackingError = "Low visual features"
                case .initializing:
                    self.trackingError = "Initializing..."
                case .relocalizing:
                    self.trackingError = "Relocalizing..."
                @unknown default:
                    self.trackingError = "Limited tracking"
                }
            }
        }
    }

    func session(_ session: ARSession, didFailWithError error: Error) {
        DispatchQueue.main.async {
            self.trackingError = error.localizedDescription
            self.isTracking = false
        }
    }
}
