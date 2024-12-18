//
//  ContentView.swift
//  RnboEmbedder
//
//  Created by Frans-Jan Wind on 18/12/2024.
//

import SwiftUI

struct ContentView: View {
    @StateObject var audioManager = AudioManager()

    var body: some View {
        VStack {
            Button("Start") {
                audioManager.start()
            }
            Button("Stop") {
                audioManager.stop()
            }
        }
    }
}
