//
//  SettingsView.swift
//  robot_arm_teleop
//
//  Created by Gemini on 3/13/26.
//

import SwiftUI

struct SettingsView: View {
    @Binding var serverURL: String
    @Binding var isPresented: Bool
    var connectAction: () -> Void
    var disconnectAction: () -> Void
    var isConnected: Bool
    var isConnecting: Bool
    var connectionError: String?

    @State private var showConnectedMessage = false

    var body: some View {
        NavigationView {
            Form {
                Section(header: Text("Network Configuration")) {
                    TextField("Server URL (e.g., ws://192.168.1.10:8000/ws)", text: $serverURL)
                        .autocapitalization(.none)
                        .disableAutocorrection(true)
                        .keyboardType(.URL)
                        .disabled(isConnecting)
                }

                Section {
                    if isConnected {
                        Button(action: {
                            disconnectAction()
                            isPresented = false
                        }) {
                            Text("Disconnect")
                                .foregroundColor(.red)
                        }
                    } else {
                        Button(action: {
                            connectAction()
                        }) {
                            HStack {
                                Text("Connect")
                                if isConnecting {
                                    Spacer()
                                    ProgressView()
                                }
                            }
                        }
                        .disabled(isConnecting)
                    }
                }

                // Connection status feedback
                if showConnectedMessage {
                    Section {
                        HStack {
                            Image(systemName: "checkmark.circle.fill")
                                .foregroundColor(.green)
                            Text("Connected successfully")
                                .foregroundColor(.green)
                        }
                    }
                }

                if let error = connectionError, !isConnecting {
                    Section {
                        HStack {
                            Image(systemName: "exclamationmark.triangle.fill")
                                .foregroundColor(.red)
                            Text(error)
                                .foregroundColor(.red)
                                .font(.subheadline)
                        }
                    }
                }
            }
            .navigationTitle("Settings")
            .toolbar {
                ToolbarItem(placement: .navigationBarTrailing) {
                    Button("Done") {
                        isPresented = false
                    }
                }
            }
            .onChange(of: isConnected) { connected in
                if connected {
                    showConnectedMessage = true
                    // Auto-dismiss after showing success
                    DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) {
                        isPresented = false
                    }
                }
            }
        }
    }
}
