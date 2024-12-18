//
//  RNBOBridge.h
//  RnboEmbedder
//
//  Created by Frans-Jan Wind on 18/12/2024.
//

#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

@interface RNBOBridge : NSObject

// Initialiseer de engine met een gegeven sample rate en block size
- (void)initializeEngineWithSampleRate:(double)sampleRate
                             blockSize:(int)blockSize;

// Verwerk audio frames
// inBuffer / outBuffer zijn pointers naar float arrays met audio samples
- (void)processWithInput:(float *)inBuffer
                  output:(float *)outBuffer
                numFrames:(UInt32)numFrames;

// Stuur een MIDI event naar de RNBO engine
- (void)sendMIDIEventWithStatus:(uint8_t)status
                          data1:(uint8_t)data1
                          data2:(uint8_t)data2
                       timestamp:(UInt32)timeStamp;

@end

NS_ASSUME_NONNULL_END
