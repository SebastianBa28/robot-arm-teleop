//
//  WebSocketManager.swift
//  robot_arm_teleop
//

import Foundation
import Combine

class WebSocketManager: ObservableObject {
    @Published var isConnected: Bool = false
    @Published var isConnecting: Bool = false
    @Published var lastFeedback: FeedbackData?
    @Published var connectionError: String?

    private var webSocketTask: URLSessionWebSocketTask?
    private let session = URLSession(configuration: .default)
    private let encoder = JSONEncoder()
    private let decoder = JSONDecoder()

    private var currentURL: String?
    private var retryCount = 0
    private let maxRetries = 3
    private var retryWorkItem: DispatchWorkItem?

    func connect(to urlString: String) {
        guard let url = URL(string: urlString) else {
            connectionError = "Invalid URL"
            return
        }

        retryWorkItem?.cancel()
        currentURL = urlString
        retryCount = 0
        connectionError = nil

        DispatchQueue.main.async {
            self.isConnecting = true
        }

        webSocketTask?.cancel(with: .normalClosure, reason: nil)
        webSocketTask = session.webSocketTask(with: url)
        webSocketTask?.resume()

        receiveMessage()

        // Verify the connection with a ping
        webSocketTask?.sendPing { [weak self] error in
            guard let self = self else { return }
            DispatchQueue.main.async {
                self.isConnecting = false
                if let error = error {
                    self.isConnected = false
                    self.connectionError = "Connection failed: \(error.localizedDescription)"
                } else {
                    self.isConnected = true
                    self.connectionError = nil
                }
            }
        }
    }

    func disconnect() {
        retryWorkItem?.cancel()
        currentURL = nil
        retryCount = 0
        webSocketTask?.cancel(with: .normalClosure, reason: nil)
        webSocketTask = nil
        DispatchQueue.main.async {
            self.isConnected = false
            self.isConnecting = false
            self.lastFeedback = nil
        }
    }

    func send(twist: TwistData) {
        guard isConnected, let task = webSocketTask else { return }

        do {
            let data = try encoder.encode(twist)
            task.send(.data(data)) { error in
                if let error = error {
                    print("Send error: \(error)")
                }
            }
        } catch {
            print("Encode error: \(error)")
        }
    }

    private func receiveMessage() {
        guard let task = webSocketTask else { return }

        task.receive { [weak self] result in
            guard let self = self else { return }

            switch result {
            case .failure(let error):
                DispatchQueue.main.async {
                    self.connectionError = error.localizedDescription
                    self.isConnected = false
                }
                self.attemptReconnect()

            case .success(let message):
                // Connection confirmed working
                DispatchQueue.main.async {
                    if !self.isConnected { self.isConnected = true }
                }
                self.retryCount = 0

                switch message {
                case .string(let text):
                    if let data = text.data(using: .utf8) {
                        self.handleFeedback(data: data)
                    }
                case .data(let data):
                    self.handleFeedback(data: data)
                @unknown default:
                    break
                }

                self.receiveMessage()
            }
        }
    }

    private func handleFeedback(data: Data) {
        do {
            let feedback = try decoder.decode(FeedbackData.self, from: data)
            DispatchQueue.main.async {
                self.lastFeedback = feedback
            }
        } catch {
            print("Decode feedback error: \(error)")
        }
    }

    private func attemptReconnect() {
        guard let url = currentURL, retryCount < maxRetries else {
            DispatchQueue.main.async {
                self.connectionError = self.retryCount >= self.maxRetries
                    ? "Failed after \(self.maxRetries) retries"
                    : self.connectionError
            }
            return
        }

        retryCount += 1
        let delay = Double(retryCount) * 2.0

        let workItem = DispatchWorkItem { [weak self] in
            self?.connect(to: url)
        }
        retryWorkItem = workItem
        DispatchQueue.main.asyncAfter(deadline: .now() + delay, execute: workItem)
    }
}
