import EventEmitter from 'events';
import HID from 'node-hid';

enum JoyConHIDIdentifiers {
  Left = 0x2006,
  Right = 0x2007,
  Pro = 0x2009,
}

enum LEDValue {
  ONE = 1,
  TWO = 2,
  THREE = 4,
  FOUR = 8,
  ONE_FLASH = 16,
  TWO_FLASH = 32,
  THREE_FLASH = 64,
  FOUR_FLASH = 128,
}

enum Subcommand {
  BluetoothManualPairing = 0x01,
  RequestDeviceInfo = 0x02,
  SetInputReportMode = 0x03,
  TriggerButtonsElapsedTime = 0x04,
  GetPageListState = 0x05,
  SetHCIState = 0x06,
  ResetPairingInfo = 0x07,
  SetLowPowerState = 0x08,
  SPIFlashRead = 0x10,
  SPIFlashWrite = 0x11,
  ResetMCU = 0x20,
  SetMCUConfiguration = 0x21,
  SetMCUState = 0x22,
  SetGPIO = 0x2A,
  SetLED = 0x30,
  GetLED = 0x31,
  SetHomeLight = 0x38,
  EnableIMU = 0x40,
  SetIMUSensitivity = 0x41,
  WriteIMU = 0x42,
  ReadIMU = 0x43,
  EnableVibration = 0x48,
  GetVoltage = 0x50,
}

// enum JoyConRightDirection {
//   LEFT = 0x00,
//   UP_LEFT = 0x01,
//   UP = 0x02,
//   UP_RIGHT = 0x03,
//   RIGHT = 0x04,
//   DOWN_RIGHT = 0x05,
//   DOWN = 0x06,
//   DOWN_LEFT = 0x07,
//   NEUTRAL = 0x08,
// }

// enum JoyConLeftDirection {
//   RIGHT = 0x00,
//   DOWN_RIGHT = 0x01,
//   DOWN = 0x02,
//   DOWN_LEFT = 0x03,
//   LEFT = 0x04,
//   UP_LEFT = 0x05,
//   UP = 0x06,
//   UP_RIGHT = 0x07,
//   NEUTRAL = 0x08,
// }

const byteToU16 = (byte: number) => byte << 8;
const byteToI16 = (byte: number) => {
  // const u16 = byteToU16(byte);
  return (byte > Math.pow(2, 15)) ? byte - Math.pow(2, 16) : byte;
};

function uint16_to_int16(number: number) {
  return byteToI16(number);
}

const _averages: { [key: string]: number[] } = {};
function rollingAverage(identifier: string, nextValue: number, length: number = 6) {
  if (!_averages.hasOwnProperty(identifier)) _averages[identifier] = [];
  _averages[identifier].push(nextValue);
  if (_averages[identifier].length > length) _averages[identifier].shift();
  return _averages[identifier].reduce((sum, n) => sum + n, 0) / _averages[identifier].length;
}

function clamp(num: number, min: number, max: number) {
  return Math.max(min, Math.min(max, num));
}

interface InputReportFull {
  buttons: number | undefined;
  buttons2: number | undefined;
  stickX: number;
  stickY: number;
  battery: number;
  accel: number[];
  gyro: number[];
}

class JoyCon {
  static HID_VENDOR_ID = 1406;
  static isJoyCon(device: HID.Device) {
    return device.vendorId === JoyCon.HID_VENDOR_ID && (
      device.productId === JoyConHIDIdentifiers.Left ||
      device.productId === JoyConHIDIdentifiers.Right ||
      device.productId === JoyConHIDIdentifiers.Pro
    );
  }
  static getAllJoyCons() {
    const devices = HID.devices();
    return devices
      .filter(device => JoyCon.isJoyCon(device));
  }

  private device: HID.Device;
  private hid: HID.HID;
  private globalPacketNumber = 0;

  private stickCalXL: number[] = [];
  private stickCalYL: number[] = [];
  private stickCalXR: number[] = [];
  private stickCalYR: number[] = [];

  private sensorCal: number[][] = [[],[],[]];
  private accCalCoeff: number[] = [];
  private gyroCalCoeff: number[] = [];
  private gyroOffsets: number[] = [0,0,0];
  private gyroOffsetsCount: number = 0;

  constructor(device: HID.Device) {
    if (!device.path) throw new Error('Error loading device');
    this.device = device;
    this.hid = new HID.HID(device.path);
    this.hid.on('data', bytes => this.handleData(bytes));

    this.hid.setNonBlocking(false);
    this.sendSubcommand(Subcommand.EnableVibration, 1);
    this.sendSubcommand(Subcommand.EnableIMU, 0x1);
    this.sendSubcommand(Subcommand.SetInputReportMode, 0x30);

    this.initializeCalibration();
  }

  private async initializeCalibration() {
    const factorySensorCal = await this.spiRead(0x6020, 0x18);
    const factoryStickCal = await this.spiRead(0x603d, 0x12);
    const sensorModel = await this.spiRead(0x6080, 0x6);
    const stickModel = await this.spiRead(0x6086, 0x12);
    const stickModel2 = await this.spiRead(0x6098, 0x12);
    const userStickCal = await this.spiRead(0x8010, 0x16);
    const userSensorCal = await this.spiRead(0x8026, 0x1a);
    if ((userSensorCal[0x0] | userSensorCal[0x1] << 8) == 0xA1B2) {
      console.log('!!! USER SENSOR CAL', userSensorCal);
    }
    if ((userStickCal[0] | userStickCal[1] << 8) == 0xA1B2) {
      console.log('!!! USER STICK CAL L');
    }
    if ((userStickCal[0xB] | userStickCal[0xC] << 8) == 0xA1B2) {
      console.log('!!! USER STICK CAL R');
    }
    if (this.isLeftJoyCon || this.isProController) {
      this.stickCalXL[1] = (factoryStickCal[4] << 8) & 0xF00 | factoryStickCal[3];
      this.stickCalYL[1] = (factoryStickCal[5] << 4) | (factoryStickCal[4] >> 4);
      this.stickCalXL[0] = this.stickCalXL[1] - ((factoryStickCal[7] << 8) & 0xF00 | factoryStickCal[6]);
      this.stickCalYL[0] = this.stickCalYL[1] - ((factoryStickCal[8] << 4) | (factoryStickCal[7] >> 4));
      this.stickCalXL[2] = this.stickCalXL[1] + ((factoryStickCal[1] << 8) & 0xF00 | factoryStickCal[0]);
      this.stickCalYL[2] = this.stickCalYL[1] + ((factoryStickCal[2] << 4) | (factoryStickCal[2] >> 4));
    }
    if (this.isRightJoyCon || this.isProController) {
      this.stickCalXR[1] = (factoryStickCal[10] << 8) & 0xF00 | factoryStickCal[9];
      this.stickCalYR[1] = (factoryStickCal[11] << 4) | (factoryStickCal[10] >> 4);
      this.stickCalXR[0] = this.stickCalXR[1] - ((factoryStickCal[13] << 8) & 0xF00 | factoryStickCal[12]);
      this.stickCalYR[0] = this.stickCalYR[1] - ((factoryStickCal[14] << 4) | (factoryStickCal[13] >> 4));
      this.stickCalXR[2] = this.stickCalXR[1] + ((factoryStickCal[16] << 8) & 0xF00 | factoryStickCal[15]);
      this.stickCalYR[2] = this.stickCalYR[1] + ((factoryStickCal[17] << 4) | (factoryStickCal[16] >> 4));
    }

    // all uint16 to int16
    this.sensorCal[0][0] = uint16_to_int16(factorySensorCal[0] | factorySensorCal[1] << 8);
    this.sensorCal[0][1] = uint16_to_int16(factorySensorCal[2] | factorySensorCal[3] << 8);
    this.sensorCal[0][2] = uint16_to_int16(factorySensorCal[4] | factorySensorCal[5] << 8);
    this.sensorCal[1][0] = uint16_to_int16(factorySensorCal[0xC] | factorySensorCal[0xD] << 8);
    this.sensorCal[1][1] = uint16_to_int16(factorySensorCal[0xE] | factorySensorCal[0xF] << 8);
    this.sensorCal[1][2] = uint16_to_int16(factorySensorCal[0x10] | factorySensorCal[0x11] << 8);

    // Use SPI calibration and convert them to SI acc unit
    this.accCalCoeff[0] = (1.0 / (16384 - uint16_to_int16(this.sensorCal[0][0]))) * 4.0 * 9.8;
    this.accCalCoeff[1] = (1.0 / (16384 - uint16_to_int16(this.sensorCal[0][1]))) * 4.0 * 9.8;
    this.accCalCoeff[2] = (1.0 / (16384 - uint16_to_int16(this.sensorCal[0][2]))) * 4.0 * 9.8;
    // Use SPI calibration and convert them to SI gyro unit
    this.gyroCalCoeff[0] = (936.0 / (13371 - uint16_to_int16(this.sensorCal[1][0]))) * 0.01745329251994;
    this.gyroCalCoeff[1] = (936.0 / (13371 - uint16_to_int16(this.sensorCal[1][1]))) * 0.01745329251994;
    this.gyroCalCoeff[2] = (936.0 / (13371 - uint16_to_int16(this.sensorCal[1][2]))) * 0.01745329251994;

    const deadzoneRead = await this.spiRead(this.isLeftJoyCon ? 0x6086 : 0x6098, 16);
    const deadzone = (deadzoneRead[4] << 8) & 0xF00 | deadzoneRead[3];

    const gyroNeutralRead = await this.spiRead(0x8034, 10);
    const gyroNeutral = [
      uint16_to_int16(gyroNeutralRead[0] | ((gyroNeutralRead[1] << 8) & 0xFF00)),
      uint16_to_int16(gyroNeutralRead[2] | ((gyroNeutralRead[3] << 8) & 0xFF00)),
      uint16_to_int16(gyroNeutralRead[4] | ((gyroNeutralRead[5] << 8) & 0xFF00)),
    ];
    console.log('gyro neutral', gyroNeutral);
  }

  get isLeftJoyCon() {
    return this.device.productId === JoyConHIDIdentifiers.Left;
  }
  get isRightJoyCon() {
    return this.device.productId === JoyConHIDIdentifiers.Right;
  }
  get isProController() {
    return this.device.productId === JoyConHIDIdentifiers.Pro;
  }

  private send(data: number[]) {
    this.globalPacketNumber = (this.globalPacketNumber + 0x1) % 0x10;
    const bytes = [...data];
    bytes[1] = this.globalPacketNumber;
    this.hid.write(bytes);
  }

  private sendSubcommand(subcommand: Subcommand, value: number) {
    const bytes = new Array(0x40).fill(0);
    bytes[0] = 0x01;
    bytes[10] = subcommand;
    bytes[11] = value;
    this.send(bytes);
  }

  private async spiRead(offset: number, readLen: number) {
    return new Promise<number[]>((resolve, reject) => {
      this.sendSubcommand(Subcommand.SPIFlashRead, offset);
      this.hid.read((err, data) => {
        if (err) return reject(err);
        resolve(data.slice(0, readLen));
      });
    });
  }

  close() {
    this.sendSubcommand(Subcommand.SetLED, 0x00);
    this.sendSubcommand(Subcommand.EnableIMU, 0x00);
    this.sendSubcommand(Subcommand.EnableVibration, 0x00);
    this.hid.close();
  }

  _buttonsFromInputReport3F(bytes: number[]) {
    if (this.isLeftJoyCon) {
      return {
        dpadLeft: Boolean(bytes[1] & 0x01),
        dpadDown: Boolean(bytes[1] & 0x02),
        dpadUp: Boolean(bytes[1] & 0x04),
        dpadRight: Boolean(bytes[1] & 0x08),

        minus: Boolean(bytes[2] & 0x01),
        screenshot: Boolean(bytes[2] & 0x20),

        sl: Boolean(bytes[1] & 0x10),
        sr: Boolean(bytes[1] & 0x20),

        l: Boolean(bytes[2] & 0x40),
        zl: Boolean(bytes[2] & 0x80),

        analogStickPress: Boolean(bytes[2] & 0x04),
        analogStick: bytes[3]
      };
    }
    if (this.isRightJoyCon) {
      return {
        a: Boolean(bytes[1] & 0x01),
        x: Boolean(bytes[1] & 0x02),
        b: Boolean(bytes[1] & 0x04),
        y: Boolean(bytes[1] & 0x08),

        plus: Boolean(bytes[2] & 0x02),
        home: Boolean(bytes[2] & 0x10),

        sl: Boolean(bytes[1] & 0x10),
        sr: Boolean(bytes[1] & 0x20),

        r: Boolean(bytes[2] & 0x40),
        zr: Boolean(bytes[2] & 0x80),

        analogStickPress: Boolean(bytes[2] & 0x08),
        analogStick: bytes[3]
      };
    }
    return {};
  }

  _readInputReport30(bytes: Buffer): InputReportFull {
    console.log(bytes);
    let buttons: number | undefined, buttons2: number | undefined;
    if (this.isLeftJoyCon) {
      buttons = (bytes[4] << 8) | (bytes[5] & 0xFF);
    } else if (this.isRightJoyCon) {
      buttons = (bytes[4] << 8) | (bytes[3] & 0xFF);
    } else if (this.isProController) {
      buttons = (bytes[4] << 8) | (bytes[5] & 0xFF);
      buttons2 = (bytes[4] << 8) | (bytes[3] & 0xFF);
      //   // fix some non-sense the Pro Controller does
      //   // clear nth bit
      //   //num &= ~(1UL << n);
      //   jc->buttons &= ~(1UL << 9);
      //   jc->buttons &= ~(1UL << 10);
      //   jc->buttons &= ~(1UL << 12);
      //   jc->buttons &= ~(1UL << 14);

      //   jc->buttons2 &= ~(1UL << 8);
      //   jc->buttons2 &= ~(1UL << 11);
      //   jc->buttons2 &= ~(1UL << 13);
    }
    const stickOffset = this.isLeftJoyCon ? 6 : this.isRightJoyCon ? 9 : 0;
    const rawStickX: number = bytes[stickOffset + 0] | ((bytes[stickOffset + 1] & 0xF) << 8);
    const rawStickY: number = (bytes[stickOffset + 1] >> 4) | (bytes[stickOffset + 2] << 4);

    const [stickX, stickY] = this.calcAnalogStick(rawStickX, rawStickY, this.isLeftJoyCon ? this.stickCalXL : this.stickCalXR, this.isLeftJoyCon ? this.stickCalYL : this.stickCalYR);
    console.log(rawStickX, rawStickY, stickX, stickY);
    // jc->CalcAnalogStick();
    // // pro controller:
    // if (jc->left_right == 3) {
    //   stick_data += 6;
    //   uint16_t stick_x = stick_data[0] | ((stick_data[1] & 0xF) << 8);
    //   uint16_t stick_y = (stick_data[1] >> 4) | (stick_data[2] << 4);
    //   jc->stick.x = (int)(unsigned int)stick_x;
    //   jc->stick.y = (int)(unsigned int)stick_y;
    //   stick_data += 3;
    //   uint16_t stick_x2 = stick_data[0] | ((stick_data[1] & 0xF) << 8);
    //   uint16_t stick_y2 = (stick_data[1] >> 4) | (stick_data[2] << 4);
    //   jc->stick2.x = (int)(unsigned int)stick_x2;
    //   jc->stick2.y = (int)(unsigned int)stick_y2;

    //   // calibration data:
    //   jc->CalcAnalogStick();
    // }
    const battery = (bytes[2] & 0xF0) >> 4;

    // m/s^2 absolute
    const accelX = uint16_to_int16(bytes[13] | (bytes[14] << 8) & 0xFF00) * this.accCalCoeff[0];
    const accelY = uint16_to_int16(bytes[15] | (bytes[16] << 8) & 0xFF00) * this.accCalCoeff[1];
    const accelZ = uint16_to_int16(bytes[17] | (bytes[18] << 8) & 0xFF00) * this.accCalCoeff[2];
    // rad/sec relative
    let roll = ((uint16_to_int16(bytes[19] | (bytes[20] << 8) & 0xFF00)) - this.sensorCal[1][0]) * this.gyroCalCoeff[0];
    let pitch = ((uint16_to_int16(bytes[21] | (bytes[22] << 8) & 0xFF00)) - this.sensorCal[1][1]) * this.gyroCalCoeff[1];
    let yaw = ((uint16_to_int16(bytes[23] | (bytes[24] << 8) & 0xFF00)) - this.sensorCal[1][2]) * this.gyroCalCoeff[2];
    if (!Number.isNaN(roll) && !Number.isNaN(pitch) && !Number.isNaN(roll)) {
      this.setGyroOffsets(roll, pitch, yaw);
      roll -= this.gyroOffsets[0];
      pitch -= this.gyroOffsets[1];
      yaw -= this.gyroOffsets[2];
    }

    return {
      buttons, buttons2, stickX, stickY, battery,
      accel: [accelX, accelY, accelZ],
      gyro: [roll, pitch, yaw],
    };
  }

  private calcAnalogStick(x: number, y: number, calX: number[], calY: number[]) {
    // buf_ = ReadSPI(0x60, (isLeft ? (byte)0x86 : (byte)0x98), 16);
    // deadzone = (UInt16)((buf_[4] << 8) & 0xF00 | buf_[3]);

    // let stickX: number = 0, stickY: number = 0;

    // const dx = x - calX[1];
    // if (Math.abs(dx) >= deadzone) {
    //   if (dx > 0) {
    //     stickX = dx / calX[2];
    //   } else {
    //     stickX = dx / calX[0];
    //   }
    // }

    // const dy = y - calY[1];
    // if (Math.abs(dy) >= deadzone) {
    //   if (dy > 0) {
    //     stickX = dy / calY[2];
    //   } else {
    //     stickX = dy / calY[0];
    //   }
    // }

    // stick_cal[isLeft ? 0 : 2] = (UInt16)((buf_[1] << 8) & 0xF00 | buf_[0]); // X Axis Max above center
    // stick_cal[isLeft ? 1 : 3] = (UInt16)((buf_[2] << 4) | (buf_[1] >> 4));  // Y Axis Max above center
    // stick_cal[isLeft ? 2 : 4] = (UInt16)((buf_[4] << 8) & 0xF00 | buf_[3]); // X Axis Center
    // stick_cal[isLeft ? 3 : 5] = (UInt16)((buf_[5] << 4) | (buf_[4] >> 4));  // Y Axis Center
    // stick_cal[isLeft ? 4 : 0] = (UInt16)((buf_[7] << 8) & 0xF00 | buf_[6]); // X Axis Min below center
    // stick_cal[isLeft ? 5 : 1] = (UInt16)((buf_[8] << 4) | (buf_[7] >> 4));  // Y Axis Min below center
    // return [
    //   stickX,
    //   stickY,
    // ];
    return [
      (x - calX[1]) / Math.abs((calX[1] < x ? calX[2] : calX[0]) - calX[1]),
      (y - calY[1]) / Math.abs((calY[1] < y ? calY[2] : calY[0]) - calY[1]),
    ];


    // const deadZoneCenter = 0.15;
    // const deadZoneOuter = 0.10;
    // console.log('n1', x, y)
    // x = clamp(x, calX[0], calX[2]);
    // y = clamp(y, calY[0], calY[2]);
    // console.log('n2', x, y)
    // let xf: number, yf: number;
    // if (x >= calX[1]) {
    //   xf = (x - calX[1]) / (calX[2] - calX[1]);
    // } else {
    //   xf = -(x - calX[1]) / (calX[0] - calX[1]);
    // }
    // if (y >= calY[1]) {
    //   yf = (y - calY[1]) / (calY[2] - calY[1]);
    // } else {
    //   yf = -(y - calY[1]) / (calY[0] - calY[1]);
    // }
    // xf = Number.isNaN(xf) ? 0 : xf;
    // yf = Number.isNaN(yf) ? 0 : yf;

    // const mag = Math.sqrt(xf * xf + yf * yf);
    // console.log(xf, yf, mag, calX, calY);
    // let outX: number, outY: number;
    // if (mag > deadZoneCenter) {
    //   const legalRange = 1.0 - deadZoneOuter - deadZoneCenter;
    //   const normalizedMag = Math.min(1.0, (mag - deadZoneCenter) / legalRange);
    //   const scale = normalizedMag / mag;
    //   outX = xf * scale;
    //   outY = yf * scale;
    // } else {
    //   outX = 0.0;
    //   outY = 0.0;
    // }
    // return [outX, outY];
  }

  private setGyroOffsets(roll: number, pitch: number, yaw: number) {
    const thresh = 0.1;
    // if (Math.abs(roll) > thresh || Math.abs(pitch) > thresh || Math.abs(yaw) > thresh) {
    //   return;
    // }
    this.gyroOffsetsCount += 1;
    this.gyroOffsets[0] += (roll - this.gyroOffsets[0]) / this.gyroOffsetsCount;
    this.gyroOffsets[1] += (pitch - this.gyroOffsets[1]) / this.gyroOffsetsCount;
    this.gyroOffsets[2] += (yaw - this.gyroOffsets[2]) / this.gyroOffsetsCount;
  }

  private handleData(bytes: Buffer) {
    if (bytes[0] === 0x30) {
      const inputs = this._readInputReport30(bytes);
      console.log(inputs);
    }
    // if (bytes[0] !== 0x3f) return;
    // const nextButtons = this._buttonsFromInputReport3F(bytes);
  }
}

export default JoyCon;
