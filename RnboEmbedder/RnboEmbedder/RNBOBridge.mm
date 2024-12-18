
#import "RNBOBridge.h"
#import "RNBO_Engine.h" // Zorg dat dit overeenkomt met de naam en locatie van je RNBO engine header

@interface RNBOBridge () {
    std::unique_ptr<RNBO_Engine> engine;
}
@end

@implementation RNBOBridge

- (instancetype)init {
    self = [super init];
    if (self) {
        engine = std::make_unique<RNBO_Engine>();
        // Als je engine extra setup nodig heeft, doe dat hier
        // Bijvoorbeeld: engine->setup();
    }
    return self;
}

- (void)initializeEngineWithSampleRate:(double)sampleRate
                             blockSize:(int)blockSize {
    if (engine) {
        // Pas deze methode aan aan de werkelijke signatuur in RNBO_Engine
        engine->initialize(sampleRate, blockSize);
    }
}

- (void)processWithInput:(float *)inBuffer
                  output:(float *)outBuffer
                numFrames:(UInt32)numFrames {
    if (engine) {
        // Ook hier pas je aan naar de werkelijke methode in RNBO_Engine
        // Vaak is dit iets als: engine->process(inBuffer, outBuffer, numFrames);
        engine->process(inBuffer, outBuffer, numFrames);
    }
}

- (void)sendMIDIEventWithStatus:(uint8_t)status
                          data1:(uint8_t)data1
                          data2:(uint8_t)data2
                       timestamp:(UInt32)timeStamp {
    if (engine) {
        // Zorg dat RNBO_Engine een geschikte methode heeft, bijvoorbeeld:
        // engine->sendMidiEvent(status, data1, data2, timeStamp);
        engine->sendMidiEvent(status, data1, data2, timeStamp);
    }
}

@end
