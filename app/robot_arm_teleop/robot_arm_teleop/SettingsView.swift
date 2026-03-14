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
    
    var body: some View {
        NavigationView {
            Form {
                Section(header: Text("Network Configuration")) {
                    TextField("Server URL (e.g., ws://192.168.1.10:8000/ws)", text: $serverURL)
                        .autocapitalization(.none)
                        .disableAutocorrection(true)
                        .keyboardType(.URL)
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
                            isPresented = false
                        }) {
                            Text("Connect")
                                .foregroundColor(.blue)
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
        }
    }
}
