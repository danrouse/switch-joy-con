import { Vector3, Quaternion, Matrix4 } from "three";
import HID from "node-hid";

enum JoyConHIDIdentifiers {
  Left = 0x2006,
  Right = 0x2007,
  Pro = 0x2009
}

export enum DebugType {
  NONE,
  ALL,
  COMMS,
  THREADING,
  IMU,
  RUMBLE
}

enum SendType {
  Subcommand = 0x01,
  Rumble = 0x10
}

enum JoyConAttachmentState {
  NOT_ATTACHED,
  DROPPED,
  NO_JOYCONS,
  ATTACHED,
  INPUT_MODE_0x30,
  IMU_DATA_OK
}

enum SPIOffset {
  User = 0x80,
  Factory = 0x60,

  UserStickCalibrationLeft = 0x12,
  UserStickCalibrationRight = 0x1d,
  FactoryStickCalibrationLeft = 0x3d,
  FactoryStickCalibrationRight = 0x46,
  StickDeadZoneLeft = 0x86,
  StickDeadZoneRight = 0x98,
  UserGyroNeutralPoint = 0x34,
  FactoryGyroNeutralPoint = 0x29,
}

enum Button {
  DPAD_DOWN = 0,
  DPAD_RIGHT = 1,
  DPAD_LEFT = 2,
  DPAD_UP = 3,
  SL = 4,
  SR = 5,
  MINUS = 6,
  HOME = 7,
  PLUS = 8,
  CAPTURE = 9,
  STICK = 10,
  SHOULDER_1 = 11,
  SHOULDER_2 = 12
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
  SetGPIO = 0x2a,
  SetLED = 0x30,
  GetLED = 0x31,
  SetHomeLight = 0x38,
  EnableIMU = 0x40,
  SetIMUSensitivity = 0x41,
  WriteIMU = 0x42,
  ReadIMU = 0x43,
  EnableVibration = 0x48,
  GetVoltage = 0x50
}

function unsigned_to_int16(byte: number) {
  return byte > Math.pow(2, 15) ? byte - Math.pow(2, 16) : byte;
}

function hex(n: any) {
  return Number(n).toString(16).padStart(2, "0")
}

function clamp(x: number, min: number, max: number) {
  if (x < min) return min;
  if (x > max) return max;
  return x;
}

class Report {
  r: Buffer | number[];
  t: Date;

  constructor(report: Buffer | number[], time: Date) {
    this.r = report;
    this.t = time;
  }

  public GetTime() {
    return this.t;
  }

  public CopyBuffer(b: Buffer | number[], report_len: number) {
    for (let i = 0; i < report_len; ++i) {
      b[i] = this.r[i];
    }
  }
}

export default class Joycon {
  public debugType: DebugType = DebugType.NONE;
  public attachmentState: JoyConAttachmentState = JoyConAttachmentState.NOT_ATTACHED;

  public buttons: { [key: number]: boolean } = {};

  public stick: number[] = [0, 0];
  private stickCal: number[] = [0, 0, 0, 0, 0, 0];
  private stickDeadZone: number = 0;

  public accel: Vector3 = new Vector3();
  public gyro: Vector3 = new Vector3();
  private gyroNeutralPoint: number[] = [0, 0, 0]; // int16

  private pollTimer: NodeJS.Timeout | undefined;
  private shouldRecenterIMU: boolean = true;
  private globalMessageCount: number = 0;
  private reports: Array<Report> = new Array<Report>();
  private prevDequeueMessageTime: number = 0;
  private prevDequeueTime: Date = new Date();
  private hidDevice: HID.Device;
  private hidHandle: HID.HID;
  private enableIMU: boolean = false;
  private enableLocalizeIMU: boolean = false;

  private _unk_filterweight: number;
  private _unk_err: number = 0;
  public _unk_i_b: Vector3 = new Vector3();
  public _unk_j_b: Vector3 = new Vector3();
  public _unk_k_b: Vector3 = new Vector3();

  constructor(
    device: HID.Device,
    enableIMU: boolean,
    enableLocalizeIMU: boolean,
    _unk_alpha: number
  ) {
    if (!device.path) throw new Error();
    this.hidDevice = device;
    this.hidHandle = new HID.HID(device.path);
    this.enableIMU = enableIMU;
    this.enableLocalizeIMU = enableLocalizeIMU;

    this._unk_filterweight = _unk_alpha;
  }

  get isLeft() {
    return this.hidDevice.productId === JoyConHIDIdentifiers.Left;
  }

  // TODO: Figure this out?
  // public GetVector() {
  //   const v1 = new Vector3(this.j_b.x, this.i_b.x, this.k_b.x);
  //   const v2 = new Vector3(this.j_b.z, this.i_b.z, this.k_b.z).negate();
  //   if (!v2.equals(new Vector3(0, 0, 0))) {
  //     return new Matrix4().lookAt(v1, v2, new Vector3(0,0,1)).quat;
  //     return new Quaternion().setFromEuler(v2.sub(v1));
  //   } else {
  //     return new Quaternion();
  //   }
  // }

  // attach/detach
  public attach(leds: number = 0x0) {
    this.attachmentState = JoyConAttachmentState.ATTACHED;
    // Input report mode
    this.sendSubcommand(Subcommand.SetInputReportMode, [0x3]);
    const { stickCal, stickDeadZone, gyroNeutralPoint } = this.getCalibration();
    this.stickCal = stickCal;
    this.stickDeadZone = stickDeadZone;
    this.gyroNeutralPoint = gyroNeutralPoint;

    // Connect
    this.sendSubcommand(Subcommand.BluetoothManualPairing, [0x01]);
    this.sendSubcommand(Subcommand.BluetoothManualPairing, [0x02]);
    this.sendSubcommand(Subcommand.BluetoothManualPairing, [0x03]);
    this.sendSubcommand(Subcommand.SetLED, [leds]);
    this.sendSubcommand(Subcommand.EnableIMU, [this.enableIMU ? 0x1 : 0x0]);
    this.sendSubcommand(Subcommand.SetInputReportMode, [0x30]);
    this.sendSubcommand(Subcommand.EnableVibration, [0x1]);
    this.debugPrint("Done with init.", DebugType.COMMS);
  }
  public detach() {
    if (this.pollTimer) clearTimeout(this.pollTimer);
    if (this.attachmentState > JoyConAttachmentState.NO_JOYCONS) {
      this.sendSubcommand(Subcommand.SetLED, [0x0]);
      this.sendSubcommand(Subcommand.EnableIMU, [0x0]);
      this.sendSubcommand(Subcommand.EnableVibration, [0x0]);
      this.sendSubcommand(Subcommand.SetInputReportMode, [0x3]);
    }
    if (this.attachmentState > JoyConAttachmentState.DROPPED) {
      this.hidHandle.close();
    }
    this.attachmentState = JoyConAttachmentState.NOT_ATTACHED;
  }

  // rumble
  private rumbleFreqLow: number = 0;
  private rumbleFreqHigh: number = 0;
  private rumbleAmplitude: number = 0.0;
  private rumbleTimeRemaining: number = 0.0;
  public setRumble(key: 'rumbleFreqLow' | 'rumbleFreqHigh' | 'rumbleAmplitude' | 'rumbleTimeRemaining', value: number) {
    if (this.attachmentState <= JoyConAttachmentState.ATTACHED) return;
    setTimeout(() => this[key] = value, Math.max(this.rumbleTimeRemaining, 0));
  }
  private updateRumble(tickRate: number) {
    if (this.rumbleTimeRemaining < 0) {
      this.rumbleFreqHigh = 160;
      this.rumbleFreqLow = 320;
      this.rumbleAmplitude = 0;
      this.rumbleTimeRemaining = 0;
    } else if (this.rumbleTimeRemaining) {
      this.rumbleTimeRemaining -= tickRate;
    }
  }

  // IMU
  private parseStick(buf: Buffer | number[]) {
    if (buf[0] == 0x00) throw new Error();
    const stickRaw = [
      buf[6 + (this.isLeft ? 0 : 3)],
      buf[7 + (this.isLeft ? 0 : 3)],
      buf[8 + (this.isLeft ? 0 : 3)]
    ];

    return this.centerSticks(
      [
        stickRaw[0] | ((stickRaw[1] & 0xf) << 8),
        (stickRaw[1] >> 4) | (stickRaw[2] << 4)
      ],
      this.stickCal,
      this.stickDeadZone
    ); // this.stick
  }
  private parseButtons(buf: Buffer | number[]) {
    if (buf[0] == 0x00) throw new Error();
    return {
      [Button.DPAD_DOWN]: (buf[3 + (this.isLeft ? 2 : 0)] & (this.isLeft ? 0x01 : 0x04)) != 0,
      [Button.DPAD_RIGHT]: (buf[3 + (this.isLeft ? 2 : 0)] & (this.isLeft ? 0x04 : 0x08)) != 0,
      [Button.DPAD_UP]: (buf[3 + (this.isLeft ? 2 : 0)] & (this.isLeft ? 0x02 : 0x02)) != 0,
      [Button.DPAD_LEFT]: (buf[3 + (this.isLeft ? 2 : 0)] & (this.isLeft ? 0x08 : 0x01)) != 0,
      [Button.HOME]: (buf[4] & 0x10) != 0,
      [Button.MINUS]: (buf[4] & 0x01) != 0,
      [Button.PLUS]: (buf[4] & 0x02) != 0,
      [Button.STICK]: (buf[4] & (this.isLeft ? 0x08 : 0x04)) != 0,
      [Button.SHOULDER_1]: (buf[3 + (this.isLeft ? 2 : 0)] & 0x40) != 0,
      [Button.SHOULDER_2]: (buf[3 + (this.isLeft ? 2 : 0)] & 0x80) != 0,
      [Button.SR]: (buf[3 + (this.isLeft ? 2 : 0)] & 0x10) != 0,
      [Button.SL]: (buf[3 + (this.isLeft ? 2 : 0)] & 0x20) != 0,
    }
  }
  private extractIMUValues(buf: Buffer | number[], n: number = 0) {
    const rawGyro = [
      unsigned_to_int16(buf[19 + n * 12] | ((buf[20 + n * 12] << 8) & 0xff00)),
      unsigned_to_int16(buf[21 + n * 12] | ((buf[22 + n * 12] << 8) & 0xff00)),
      unsigned_to_int16(buf[23 + n * 12] | ((buf[24 + n * 12] << 8) & 0xff00))
    ];
    const rawAccel = [
      unsigned_to_int16(buf[13 + n * 12] | ((buf[14 + n * 12] << 8) & 0xff00)),
      unsigned_to_int16(buf[15 + n * 12] | ((buf[16 + n * 12] << 8) & 0xff00)),
      unsigned_to_int16(buf[17 + n * 12] | ((buf[18 + n * 12] << 8) & 0xff00))
    ];
    const accel = new Vector3(...rawAccel.map(n => n * 0.00025));
    const gyro = new Vector3(...rawGyro.map((n, i) => (n - this.gyroNeutralPoint[i]) * 0.00122187695));
    return [accel, gyro];
  }
  private processIMU(buf: Buffer | number[], lastMessageTime: number) {
    // Direction Cosine Matrix method
    // http://www.starlino.com/dcm_tutorial.html

    if (
      !this.enableIMU ||
      this.attachmentState < JoyConAttachmentState.IMU_DATA_OK
    )
      return -1;

    if (buf[0] != 0x30) return -1; // no gyro data

    // read raw IMU values
    let dt = buf[1] - lastMessageTime;
    if (buf[1] < lastMessageTime) dt += 0x100;

    for (let n = 0; n < 3; ++n) {
      const [accel, gyro] = this.extractIMUValues(buf, n);

      if (this.isLeft) {
        gyro.y *= -1;
        gyro.z *= -1;
        accel.y *= -1;
        accel.z *= -1;
      }
      this.accel = accel;
      this.gyro = gyro;

      if (this.shouldRecenterIMU) {
        this._unk_i_b = new Vector3(1, 0, 0);
        this._unk_j_b = new Vector3(0, 1, 0);
        this._unk_k_b = new Vector3(0, 0, 1);
        this.shouldRecenterIMU = false;
      } else {
        const dt_sec = 0.005 * dt;
        const k_acc = this.accel.normalize().negate();
        const w_a = this._unk_k_b.cross(k_acc);
        const w_g = this.gyro.multiplyScalar(dt_sec).negate(); // check order of operations?
        const d_theta = w_a.add(w_g).multiplyScalar(this._unk_filterweight).divideScalar(1 + this._unk_filterweight);
        this._unk_k_b = this._unk_k_b.add(d_theta.cross(this._unk_k_b));
        this._unk_i_b = this._unk_i_b.add(d_theta.cross(this._unk_i_b));
        this._unk_j_b = this._unk_j_b.add(d_theta.cross(this._unk_j_b));
        //Correction, ensure new axes are orthogonal
        this._unk_err = this._unk_i_b.dot(this._unk_j_b) * 0.5;
        const i_b_ = this._unk_i_b.subScalar(this._unk_err).multiply(this._unk_j_b).normalize();
        this._unk_j_b = this._unk_j_b.subScalar(this._unk_err).multiply(this._unk_i_b).normalize();
        this._unk_i_b = i_b_;
        this._unk_k_b = this._unk_i_b.cross(this._unk_j_b);
      }
      dt = 1;
    }
    return buf[1] + 2;
  }
  public Recenter() {
    this.shouldRecenterIMU = true;
  }

  // queue
  private ReceiveRaw(prevEnqueueMessageTime: number) {
    if (!this.hidHandle) return -2;
    this.hidHandle.setNonBlocking(false);
    try {
      const ret = this.hidHandle.readSync();
      this.reports.push(new Report(ret, new Date()));
      if (prevEnqueueMessageTime == ret[1]) {
        this.debugPrint(
          `Duplicate timestamp enqueued. TS: ${hex(prevEnqueueMessageTime)}`,
          DebugType.THREADING
        );
      }
      this.debugPrint(
        `Enqueue. Bytes read: ${ret}. Timestamp: ${hex(ret[1])}`,
        DebugType.THREADING
      );
      return ret[1];
    } catch (e) {
      return 0;
    }
  }
  public readReportsQueue() {
    if (this.attachmentState > JoyConAttachmentState.NO_JOYCONS) {
      let lastMessageTime = 0;
      while (this.reports.length > 0) {
        const rep: Report = this.reports.shift() as Report;
        if (this.enableIMU) {
          if (this.enableLocalizeIMU) {
            lastMessageTime = this.processIMU(rep.r, lastMessageTime);
          } else {
            const [accel, gyro] = this.extractIMUValues(rep.r, 0);
            this.accel = accel;
            this.gyro = gyro;
          }
        }
        if (this.prevDequeueMessageTime == rep.r[1]) {
          this.debugPrint(
            `Duplicate timestamp dequeued. TS: ${hex(this.prevDequeueMessageTime)}`,
            DebugType.THREADING
          );
        }
        this.prevDequeueMessageTime = rep.r[1];
        this.debugPrint(
          `Dequeue. Queue length: ${
            this.reports.length
          }. Packet ID: ${hex(rep.r[0])}. ` +
            `Timestamp: ${hex(rep.r[1])}. Lag to dequeue: ${Date.now() -
              rep.GetTime().getTime()}. ` +
            `Lag between packets (expect 15ms): ${rep.GetTime().getTime() -
              this.prevDequeueTime.getTime()}`,
          DebugType.THREADING
        );
        this.prevDequeueTime = rep.GetTime();
        if (this.reports.length === 1) {
          this.stick = this.parseStick(rep.r);
          this.buttons = this.parseButtons(rep.r);
        }
      }
    }
  }
  public getRumblePacket(freqLo: number, freqHi: number, amplitude: number) {
    const packet: number[] = [0x0, 0x1, 0x40, 0x40];
    if (amplitude !== 0) {
      freqLo = clamp(freqLo, 40.875885, 626.286133);
      amplitude = clamp(amplitude, 0.0, 1.0);
      freqHi = clamp(freqHi, 81.75177, 1252.572266);
      const hf = (Math.round(32 * Math.log2(freqHi * 0.1)) - 0x60) * 4;
      const lf = Math.round(32 * Math.log2(freqLo * 0.1)) - 0x40;
      let hf_amp;
      if (amplitude == 0) hf_amp = 0;
      else if (amplitude < 0.117)
        hf_amp =
          (Math.log2(amplitude * 1000) * 32 - 0x60) /
            (5 - Math.pow(amplitude, 2)) -
          1;
      else if (amplitude < 0.23)
        hf_amp = Math.log2(amplitude * 1000) * 32 - 0x60 - 0x5c;
      else hf_amp = (Math.log2(amplitude * 1000) * 32 - 0x60) * 2 - 0xf6;

      let lf_amp = Math.floor(Math.round(hf_amp) * 0.5);
      const parity = lf_amp % 2;
      if (parity > 0) {
        --lf_amp;
      }

      lf_amp = Math.floor(lf_amp >> 1);
      lf_amp += 0x40;
      if (parity > 0) lf_amp |= 0x8000;
      packet[0] = hf & 0xff;
      packet[1] = (hf >> 8) & 0xff;
      packet[2] = lf;
      packet[1] += hf_amp;
      packet[2] += (lf_amp >> 8) & 0xff;
      packet[3] += lf_amp & 0xff;
    }
    return packet.concat(packet);
  }


  // poll
  public beginPolling() {
    this.poll();
    const tickRate = 1000/60;
    setInterval(() => {
      this.readReportsQueue();
      this.updateRumble(tickRate);
      console.log("buttons", this.buttons);
      console.log("stick", this.stick);
      console.log("accel", "\n", this.accel.x, "\n", this.accel.y, "\n", this.accel.z);
      console.log("gyro", "\n", this.gyro.x, "\n", this.gyro.y, "\n", this.gyro.z);
    }, tickRate);
  }
  private async poll(attempt: number = 0, prevEnqueueMessageTime: number = 0) {
    if (this.attachmentState > JoyConAttachmentState.NO_JOYCONS) {
      if (this.rumbleTimeRemaining !== 0) {
        this.sendRaw(this.getRumblePacket(this.rumbleFreqLow, this.rumbleFreqHigh, this.rumbleAmplitude), SendType.Rumble);
      }
      let messageTime = this.ReceiveRaw(prevEnqueueMessageTime);
      messageTime = this.ReceiveRaw(messageTime);
      if (messageTime > 0) {
        this.attachmentState = JoyConAttachmentState.IMU_DATA_OK;
        this.pollTimer = setTimeout(() => this.poll(0), 5);
      } else if (attempt > 1000) {
        this.attachmentState = JoyConAttachmentState.DROPPED;
        this.debugPrint("Connection lost. Is the Joy-Con connected?",DebugType.ALL);
      } else {
        this.debugPrint("Pause 5ms", DebugType.THREADING);
        this.pollTimer = setTimeout(() => this.poll(attempt + 1), 5);
      }
    }
  }

  // Calibration
  private getCalibration() {
    let buf_ = this.readSPI(SPIOffset.User, this.isLeft ? SPIOffset.UserStickCalibrationLeft : SPIOffset.UserStickCalibrationRight, 9); // get user calibration data if possible
    let found = false;
    for (let i = 0; i < 9; ++i) {
      if (buf_[i] != 0xff) {
        console.debug("Using user stick calibration data.");
        found = true;
        break;
      }
    }
    if (!found) {
      console.debug("Using factory stick calibration data.");
      buf_ = this.readSPI(SPIOffset.Factory, this.isLeft ? SPIOffset.FactoryStickCalibrationLeft : SPIOffset.FactoryStickCalibrationRight, 9); // get user calibration data if possible
    }
    const stickCal = [];
    stickCal[this.isLeft ? 0 : 2] = ((buf_[1] << 8) & 0xf00) | buf_[0]; // X Axis Max above center
    stickCal[this.isLeft ? 1 : 3] = (buf_[2] << 4) | (buf_[1] >> 4); // Y Axis Max above center
    stickCal[this.isLeft ? 2 : 4] = ((buf_[4] << 8) & 0xf00) | buf_[3]; // X Axis Center
    stickCal[this.isLeft ? 3 : 5] = (buf_[5] << 4) | (buf_[4] >> 4); // Y Axis Center
    stickCal[this.isLeft ? 4 : 0] = ((buf_[7] << 8) & 0xf00) | buf_[6]; // X Axis Min below center
    stickCal[this.isLeft ? 5 : 1] = (buf_[8] << 4) | (buf_[7] >> 4); // Y Axis Min below center

    this.debugPrintArray(stickCal, undefined, 6, 0, "Stick calibration data:");

    buf_ = this.readSPI(SPIOffset.Factory, this.isLeft ? SPIOffset.StickDeadZoneLeft : SPIOffset.StickDeadZoneRight, 16);
    const stickDeadZone = ((buf_[4] << 8) & 0xf00) | buf_[3];

    buf_ = this.readSPI(SPIOffset.User, SPIOffset.UserGyroNeutralPoint, 10);
    let gyroNeutralPoint = [
      unsigned_to_int16(buf_[0] | ((buf_[1] << 8) & 0xff00)),
      unsigned_to_int16(buf_[2] | ((buf_[3] << 8) & 0xff00)),
      unsigned_to_int16(buf_[4] | ((buf_[5] << 8) & 0xff00)),
    ];
    this.debugPrintArray(gyroNeutralPoint, DebugType.IMU, 3, 0, "User gyro neutral position:");

    // This is an extremely messy way of checking to see whether there is user stick calibration data present, but I've seen conflicting user calibration data on blank Joy-Cons. Worth another look eventually.
    if (
      gyroNeutralPoint[0] + gyroNeutralPoint[1] + gyroNeutralPoint[2] == -3 ||
      Math.abs(gyroNeutralPoint[0]) > 100 ||
      Math.abs(gyroNeutralPoint[1]) > 100 ||
      Math.abs(gyroNeutralPoint[2]) > 100
    ) {
      buf_ = this.readSPI(SPIOffset.Factory, SPIOffset.FactoryGyroNeutralPoint, 10);
      gyroNeutralPoint = [
        unsigned_to_int16(buf_[3] | ((buf_[4] << 8) & 0xff00)),
        unsigned_to_int16(buf_[5] | ((buf_[6] << 8) & 0xff00)),
        unsigned_to_int16(buf_[7] | ((buf_[8] << 8) & 0xff00)),
      ];
      this.debugPrintArray(gyroNeutralPoint, DebugType.IMU, 3, 0, "Factory gyro neutral position:");
    }
    return { stickCal, stickDeadZone, gyroNeutralPoint };
  }
  private centerSticks(vals: number[], calibration: number[], deadZone: number) {
    const s = [0, 0];
    for (let i = 0; i < 2; ++i) {
      const diff = vals[i] - calibration[2 + i];
      if (Math.abs(diff) < deadZone) {
        vals[i] = 0;
      } else if (diff > 0) {
        // if axis is above center
        s[i] = diff / calibration[i];
      } else {
        s[i] = diff / calibration[4 + i];
      }
    }
    return s;
  }

  // debug
  public debugPrint(s: string, d: DebugType) {
    if (this.debugType == DebugType.NONE) return;
    if (
      d == DebugType.ALL ||
      d == this.debugType ||
      this.debugType == DebugType.ALL
    ) {
      console.debug(s);
    }
  }
  private debugPrintArray<T>(
    arr: T[],
    d: DebugType = DebugType.NONE,
    len: number = 0,
    start: number = 0,
    format: string = ""
  ) {
    if (d != this.debugType && this.debugType != DebugType.ALL) return;
    if (len == 0) len = arr.length;
    let tostr = "";
    for (let i = 0; i < len; ++i) {
      tostr += hex(arr[i + start]);
    }
    this.debugPrint(format + tostr, d);
  }

  // HCI
  private sendRaw(data: number[], sendType: number) {
    const buf = [sendType, this.globalMessageCount, ...data];
    if (this.globalMessageCount == 0xf) this.globalMessageCount = 0;
    else ++this.globalMessageCount;
    this.debugPrintArray(buf, DebugType.COMMS, undefined, undefined, "Data sent: ");
    this.hidHandle.write(buf);
  }
  private sendSubcommand(command: Subcommand, data: Buffer | number[]) {
    const buf_: number[] = [0x00, 0x01, 0x40, 0x40, 0x00, 0x01, 0x40, 0x40, command, ...data];
    this.sendRaw(buf_, SendType.Subcommand);
    const response = this.hidHandle.readTimeout(50);
    if (!response) this.debugPrint("No response.", DebugType.COMMS);
    else
      this.debugPrintArray(response, DebugType.COMMS, 48, 1, "Response ID 0x" + hex(response[0]) + ". Data: 0x");
    return response;
  }
  private readSPI(addr1: number, addr2: number, len: number) {
    let buf: number[] = [];
    for (let i = 0; i < 100; ++i) {
      buf = this.sendSubcommand(Subcommand.SPIFlashRead, [addr2, addr1, 0x00, 0x00, len]);
      if (buf[15] == addr2 && buf[16] == addr1) break;
    }
    const read_buf = buf.slice(20, 20 + len);
    this.debugPrintArray(read_buf, DebugType.COMMS, len);
    return read_buf;
  }
}
