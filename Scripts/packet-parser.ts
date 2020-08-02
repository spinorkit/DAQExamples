import {
   IDuplexStream,
   IDataSink,
   TimePoint,
   FirstSampleRemoteTime
} from './device-api';
//import { assert } from 'libs/utility/assert';

export enum SamplingState {
   kIdle,
   kSampling
}

const enum PacketType {
   kNotFound = 0 | 0,
   kData = 1 | 0,
   kTime = 2 | 0,
   kFirstSampleTime = 3 | 0
}

//Don't fire notifications into Lightning too often!
const kMinimumSamplingUpdatePeriodms = 50;
const kGetRemoteTimeTimeoutms = 500;

const kPacketHeaderSizeBytes = 3; //not including the packet number
const kPacketStartByte = 0x50; //'P'
const kPacket2ndByte = 0xa0;

const kMaxPacketSizeBytes = 9;

function packetTypeToSize(type: PacketType) {
   switch (type) {
      case PacketType.kData:
         return 8;
      case PacketType.kTime:
         return 9;
      case PacketType.kFirstSampleTime:
         return 8;
   }
   return 0;
}

class BufferWithLen {
   constructor(
      public buf: Buffer,
      public len = 0 //we are not always using the full buffer.length
   ) {}

   setNewBuffer(buffer: Buffer) {
      this.buf = buffer;
      this.len = buffer.length;
   }

   appendFrom(other: Buffer, desired: number) {
      const sourceEnd = Math.min(desired, other.length);
      const nCopied = other.copy(this.buf, this.len, 0, sourceEnd);
      this.len += nCopied;
      return nCopied;
   }

   copyFrom(other: Buffer, sourceStart: number) {
      //const sourceEnd = Math.min(desired, other.length);
      const nCopied = other.copy(this.buf, 0, sourceStart);
      this.len = nCopied;
      return nCopied;
   }
}

export enum ParserState {
   kNoHeaderBytes,
   k1HeaderByte,
   k2HeaderBytes,
   kHasHeader,
   kHasExpectedPacket,
   kUnexpectedPacket,
   kError
}

export class Parser {
   parserState: ParserState;
   packetType: PacketType;
   packetPayloadSize = 0 | 0;
   samplingState: SamplingState = SamplingState.kIdle;

   expectedPacketCount: number; // is one byte (0 - 255)
   unexpectedPacketCount: number; //store the last unexpected count so we can resynch

   //We use double buffering to handle cases where a packet spans more that one chunk
   lastBuf: BufferWithLen;
   //bufInfo: BufferInfo;

   proxyDevice: IDataSink | null = null;

   lastError = '';
   lastUpdateTimems: number;

   //Round-trip time synchronization support (optional but recommended)
   timePoint: TimePoint | undefined;
   remoteTimeTimeoutId: NodeJS.Timeout;
   getRemoteTimeCommand: Uint8Array;
   expectedTimeRequestNumber: number;

   constructor(public inStream: IDuplexStream) {
      this.parserState = ParserState.kNoHeaderBytes;
      this.packetType = PacketType.kNotFound;
      this.packetPayloadSize = 0 | 0;
      this.expectedPacketCount = 0;
      this.unexpectedPacketCount = 0;

      //getRemoteTime() support
      this.getRemoteTimeCommand = new Uint8Array([
         'n'.charCodeAt(0),
         0,
         '\n'.charCodeAt(0)
      ]);
      this.expectedTimeRequestNumber = this.getRemoteTimeCommand[1];

      //this.bufInfo = new BufferInfo(Buffer.alloc(kMaxPacketSizeBytes));
      this.lastBuf = new BufferWithLen(Buffer.alloc(kMaxPacketSizeBytes));

      //node streams default to 'utf8' encoding, which most devices won't understand.
      //With 'utf8' encoding, non-ascii chars, such as:
      //devStream.write('\xD4\x02\x02\xD4\x76\x0A\x62');
      //could be expanded into multiple bytes, so we use 'binary' instead.
      this.inStream.setDefaultEncoding('binary');

      this.inStream.on('error', this.onError);
      // do we want to do this here? (switches stream into flowing mode).
      this.inStream.on('data', this.onData);

      this.lastUpdateTimems = performance.now();
      //'now' coommand is 'n' char followed by time request number to avoid potentially matching
      //on a response to a previous request that has been buffer in a queue somewhere
   }

   incrementTimeRequestNumber() {
      this.expectedTimeRequestNumber = this.getRemoteTimeCommand[1];
      this.getRemoteTimeCommand[1]++;
      //Avoid sending the command terminator linefeed char ('\n') used in the Arduino firmware
      if (this.getRemoteTimeCommand[1] === '\n'.charCodeAt(0)) {
         this.getRemoteTimeCommand[1]++;
      }
   }

   incrementExpectedPacketCount() {
      this.expectedPacketCount++;
      this.expectedPacketCount &= 255;
      //++gSampleCountForTesting;
   }

   isSampling(): boolean {
      return (
         SamplingState.kIdle < this.samplingState &&
         this.samplingState <= SamplingState.kSampling
      );
   }

   onError = (err: Error) => {
      this.lastError = err.message;
      console.warn(err);
   };

   setProxyDevice(proxyDevice: IDataSink | null) {
      this.proxyDevice = proxyDevice;
   }

   startSampling(): boolean {
      if (!this.inStream || !this.proxyDevice) {
         return false;
      }

      this.inStream.write('b\n'); // Arduino begin sampling command
      this.inStream.write('f\n'); // Optional - request first sample time packet
      this.samplingState = SamplingState.kSampling;

      //Comment these two lines to support handling time packets while not sampling!
      //this.expectedPacketCount = 0;
      //this.lastBuf.len = 0;

      this.proxyDevice.onSamplingStarted();

      this.lastUpdateTimems = performance.now();
      return true;
   }

   stopSampling(): boolean {
      this.samplingState = SamplingState.kIdle;
      if (!this.inStream) return false; // Can't sample if no hardware connection

      this.inStream.write('s\n'); // OpenBCI begin sampling command
      if (this.proxyDevice) this.proxyDevice.onSamplingStopped(''); // Normal user stop
      return true;
   }

   //returns the number of bytes processed from start of buffer, i.e. not from offset.
   processBuffer(buffer: Buffer, start: number, end: number) {
      let i = start;
      for (; i < end; ++i) {
         const byte = buffer[i];
         switch (this.parserState) {
            case ParserState.kNoHeaderBytes:
               if (byte === kPacketStartByte) {
                  this.parserState = ParserState.k1HeaderByte;
               }
               break;
            case ParserState.k1HeaderByte:
               if (byte === kPacket2ndByte) {
                  this.parserState = ParserState.k2HeaderBytes;
               } else {
                  this.parserState = ParserState.kNoHeaderBytes;
                  --i; //retry this byte
               }
               break;
            case ParserState.k2HeaderBytes:
               if ((byte & 0xf0) === 0x40) {
                  this.parserState = ParserState.kHasHeader;
                  switch (byte) {
                     case 0x44: //'D'
                        this.packetType = PacketType.kData;
                        break;
                     case 0x4e: //'N'
                        this.packetType = PacketType.kTime;
                        break;
                     case 0x46: //'F'
                        this.packetType = PacketType.kFirstSampleTime;
                        break;
                     default:
                        this.packetType = PacketType.kNotFound;
                        this.parserState = ParserState.kNoHeaderBytes;
                        break;
                  }
               } else {
                  this.parserState = ParserState.kNoHeaderBytes;
                  --i; //retry this byte
               }
               break;
            case ParserState.kHasHeader:
               if (byte === this.expectedPacketCount) {
                  this.parserState = ParserState.kHasExpectedPacket;
               } else {
                  this.parserState = ParserState.kUnexpectedPacket;
                  this.unexpectedPacketCount = byte;
                  const lostSamples = (byte - this.expectedPacketCount) & 255;
                  console.warn('Device lost packets:', lostSamples);
               }
               break;
            case ParserState.kUnexpectedPacket:
            case ParserState.kHasExpectedPacket:
               this.packetPayloadSize =
                  packetTypeToSize(this.packetType) -
                  kPacketHeaderSizeBytes -
                  1; //don't include packet count byte
               if (end - i < this.packetPayloadSize) {
                  //need more data before we can process the packet
                  return i;
               }
               const payloadStart = i;
               const payloadEnd = i + this.packetPayloadSize;
               if (!this.processPacketPayload(buffer.slice(i, payloadEnd))) {
                  i = payloadStart; //Start searching for a good packet!
               } else {
                  i = payloadEnd; //Finished this packet
               }
               this.packetType = PacketType.kNotFound;
               this.parserState = ParserState.kNoHeaderBytes;
               --i; //compensate for the for loop ++i that is about to happen before we go around again!
         } //switch
      } //for
      return i;
   }

   onData = (newBytes: Buffer) => {
      if (!newBytes.length) {
         return;
      }

      let offsetInNew = 0;

      const unprocessed = this.lastBuf.len;
      if (unprocessed) {
         // assert(
         //    this.parserState === ParserState.kHasExpectedPacket ||
         //       this.parserState === ParserState.kUnexpectedPacket
         // );
         if (
            this.parserState === ParserState.kHasExpectedPacket ||
            this.parserState === ParserState.kUnexpectedPacket
         ) {
            //copy new bytes required to complete the current packet
            const desired = this.packetPayloadSize - unprocessed;
            //returns the number of bytes appended
            offsetInNew = this.lastBuf.appendFrom(newBytes, desired);
            if (this.lastBuf.len < this.packetPayloadSize) return; //need more data
         }
         const processed = this.processBuffer(
            this.lastBuf.buf,
            0,
            this.lastBuf.len
         );
         //assert(processed === this.lastBuf.len);
         if (processed < this.lastBuf.len) {
            //need more input to finish processing the last packet
            return;
         }
         //Finished processing previous data
         this.lastBuf.len = 0;
      }

      const processed = this.processBuffer(
         newBytes,
         offsetInNew,
         newBytes.length
      );

      if (processed < newBytes.length) {
         //need more input to finish processing the last packet.
         //copy the unprocessed bytes to the start of lastBuf
         this.lastBuf.copyFrom(newBytes, processed);
      }

      if (this.isSampling()) {
         const now_ms = performance.now();
         if (now_ms - this.lastUpdateTimems > kMinimumSamplingUpdatePeriodms) {
            this.lastUpdateTimems = now_ms;
            if (this.proxyDevice) this.proxyDevice.onSamplingUpdate();
         }
      }
   };

   processPacketPayload(data: Buffer) {
      if (!this.proxyDevice) {
         this.lastError =
            'Device parser processPacketPayLoad() called with no proxyDevice';
         console.warn(this.lastError);
         return true;
      }

      let ok = false;
      switch (this.packetType) {
         case PacketType.kData:
            ok = this.processDataPacketPayload(data);
            break;
         case PacketType.kTime:
            ok = this.processTimePacketPayload(data);
            break;
         case PacketType.kFirstSampleTime:
            ok = this.processFirstSampleTimePacketPayload(data);
            break;
         default:
            console.error(
               'processPacketPayLoad: unknown packet type: ',
               this.packetType
            );
            ok = false; //start searching for a new packet header
            break;
      }
      if (this.parserState === ParserState.kUnexpectedPacket) {
         this.expectedPacketCount = this.unexpectedPacketCount; //attempt to resynch
      }

      this.incrementExpectedPacketCount();

      return ok;
   }

   //returns true if request is initiated, false otherwise.
   getRemoteTime(): boolean {
      if (this.timePoint) {
         console.warn('previous request still in progress!');
         return false; //previous request still in progress!
      }

      this.remoteTimeTimeoutId = global.setTimeout(() => {
         if (this.proxyDevice) {
            const error = new Error('getRemoteTime()');
            error.name = 'Timeout';
            this.timePoint = undefined;
            this.proxyDevice.onRemoteTimeEvent(error, null);
         }
      }, kGetRemoteTimeTimeoutms);

      this.timePoint = new TimePoint();

      this.inStream.write(this.getRemoteTimeCommand); // ask the device for its "now" time tick from the timer that drives the ADC
      if (this.inStream.source)
         this.inStream.source.getLocalSteadyClockTickNow(
            this.timePoint.localPreTimeTick
         );
      this.incrementTimeRequestNumber();
      return true;
   }

   processTimePacketPayload(data: Buffer) {
      if (this.parserState === ParserState.kUnexpectedPacket) {
         return false;
      }

      if (data[0] !== this.expectedTimeRequestNumber) {
         return false;
      }

      global.clearTimeout(this.remoteTimeTimeoutId);

      const timePoint = this.timePoint;
      this.timePoint = undefined; //enable a new getRemoteTime() request

      if (timePoint) {
         if (this.inStream.source)
            this.inStream.source.getLocalSteadyClockTickNow(
               timePoint.localPostTimeTick
            );

         //We assume the incoming int64 is encoded as little endian in the byte stream, i.e.
         //least signficant byte first.
         timePoint.remoteTimeTick[1] = 0; //Arduino micros() returns 32 bit time

         timePoint.remoteTimeTick[0] =
            data[1] + (data[2] << 8) + (data[3] << 16) + (data[4] << 24);

         if (this.proxyDevice) {
            this.proxyDevice.onRemoteTimeEvent(null, timePoint);
         }
      }
      return true;
   }

   processFirstSampleTimePacketPayload(data: Buffer) {
      if (this.parserState === ParserState.kUnexpectedPacket) {
         return false;
      }

      const remoteFirstSampleTime = new FirstSampleRemoteTime();

      //We assume the incoming int64 is encoded as little endian in the byte stream, i.e.
      //least signficant byte first.
      remoteFirstSampleTime.remoteFirstSampleTick[1] = 0;

      remoteFirstSampleTime.remoteFirstSampleTick[0] =
         data[0] + (data[1] << 8) + (data[2] << 16) + (data[3] << 24);

      if (this.proxyDevice) {
         this.proxyDevice.onRemoteTimeEvent(null, remoteFirstSampleTime);
      }

      if (!this.proxyDevice) {
         return true;
      }
      return true;
   }

   processDataPacketPayload(data: Buffer) {
      // assert(
      //    this.parserState === ParserState.kHasExpectedPacket ||
      //       this.parserState === ParserState.kUnexpectedPacket
      // );
      if (
         this.parserState !== ParserState.kHasExpectedPacket &&
         this.parserState !== ParserState.kUnexpectedPacket
      ) {
         return false;
      }

      if (!this.proxyDevice) {
         return true;
      }

      const outStreamBuffers = this.proxyDevice.outStreamBuffers;
      const nStreams = outStreamBuffers.length;

      //Insert invalid samples if packets missing
      if (this.parserState === ParserState.kUnexpectedPacket) {
         const lostSamples =
            (this.unexpectedPacketCount - this.expectedPacketCount) & 255;
         if (lostSamples) {
            if (this.isSampling()) {
               for (let i = 0; i < lostSamples; ++i) {
                  for (
                     let streamIndex = 0;
                     streamIndex < nStreams;
                     ++streamIndex
                  ) {
                     const outStreamBuffer = outStreamBuffers[streamIndex];
                     if (!outStreamBuffer) {
                        continue;
                     } // Don't produce data for disabled streams.

                     outStreamBuffers[streamIndex].writeInt(0x8000); //Insert 'out of range' values
                  }
               }
            }
         }
      }

      //Now add the data from the latest packet
      if (this.isSampling()) {
         let byteIndex = 0; //kStartOfDataIndex;
         for (
            let streamIndex = 0;
            streamIndex < nStreams;
            ++streamIndex, byteIndex += 2
         ) {
            const outStreamBuffer = outStreamBuffers[streamIndex];
            if (!outStreamBuffer) {
               continue;
            } // Don't produce data for disabled streams.

            // The Arduino format is little endian 16 bit.
            const value = data[byteIndex] + (data[byteIndex + 1] << 8);

            outStreamBuffers[streamIndex].writeInt(value);
         }
      }
      return true;
   }
}
