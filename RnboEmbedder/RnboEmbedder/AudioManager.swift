//
//  AudioManager.swift
//  RnboEmbedder
//
//  Created by Frans-Jan Wind on 18/12/2024.
//

import Foundation

class AudioManager: ObservableObject {
    let rnboBridge = RNBOBridge()

    init() {
        // Initialiseer je engine
        rnboBridge.initializeEngine(withSampleRate: 44100.0, blockSize: 512)
    }

    func start() {
        // Start je audio engine, sequencer, etc.
        // Tijdens het renderen kun je rnboBridge.process(...) aanroepen
    }

    func stop() {
        // Stop je audio engine
    }

    func sendMidiEvent(status: UInt8, data1: UInt8, data2: UInt8) {
        rnboBridge.sendMIDIEvent(withStatus: status, data1: data1, data2: data2, timestamp: 0)
    }
}
