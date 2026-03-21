//
//  SettingsView.swift
//  robot_arm_teleop
//

import SwiftUI

struct SettingsView: View {
    @Binding var serverURL: String
    @Binding var controlMode: String
    @Binding var axisMapX: String
    @Binding var axisMapY: String
    @Binding var axisMapZ: String
    @Binding var showFeedback: Bool
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

                Section(header: Text("Control Mode")) {
                    Picker("Mode", selection: $controlMode) {
                        Text("Velocity").tag("velocity")
                        Text("Position").tag("position")
                    }
                    .pickerStyle(.segmented)
                }

                Section(header: Text("Display")) {
                    Toggle("Show Feasibility & Manipulability", isOn: $showFeedback)
                }

                Section(header: Text("Axis Mapping (Phone → Robot)")) {
                    axisPicker(label: "Robot X ←", selection: $axisMapX)
                    axisPicker(label: "Robot Y ←", selection: $axisMapY)
                    axisPicker(label: "Robot Z ←", selection: $axisMapZ)
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
                    DispatchQueue.main.asyncAfter(deadline: .now() + 1.0) {
                        isPresented = false
                    }
                }
            }
        }
    }

    private func axisPicker(label: String, selection: Binding<String>) -> some View {
        Picker(label, selection: selection) {
            ForEach(AxisSource.allCases) { source in
                Text(source.rawValue).tag(source.rawValue)
            }
        }
    }
}
