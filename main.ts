/**
 * microbit_car
 */
//% weight=100 color=#0fbc11 icon="\uf021"
namespace microbit_carxgp {
    let _DEBUG: boolean = false

    let CHIP_ADDR = 64
    let freq = 50
    const MIN_CHIP_ADDRESS = 0x40
    const MAX_CHIP_ADDRESS = MIN_CHIP_ADDRESS + 62
    const chipResolution = 4096
    const PrescaleReg = 0xFE //the prescale register address
    const modeRegister1 = 0x00 // MODE1
    const modeRegister1Default = 0x01
    const modeRegister2 = 0x01 // MODE2
    const modeRegister2Default = 0x04
    const sleep = modeRegister1Default | 0x10; // Set sleep bit to 1
    const wake = modeRegister1Default & 0xEF; // Set sleep bit to 0
    const restart = wake | 0x80; // Set restart bit to 1
    const allChannelsOnStepLowByte = 0xFA // ALL_LED_ON_L
    const allChannelsOnStepHighByte = 0xFB // ALL_LED_ON_H
    const allChannelsOffStepLowByte = 0xFC // ALL_LED_OFF_L
    const allChannelsOffStepHighByte = 0xFD // ALL_LED_OFF_H
    const PinRegDistance = 4
    const channel0OnStepLowByte = 0x06 // LED0_ON_L
    const channel0OnStepHighByte = 0x07 // LED0_ON_H
    const channel0OffStepLowByte = 0x08 // LED0_OFF_L
    const channel0OffStepHighByte = 0x09 // LED0_OFF_H

    const hexChars = ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f']

    export enum PinNum {
        Pin0 = 0,
        Pin1 = 1,
        Pin2 = 2,
        Pin3 = 3,
        Pin4 = 4,
        Pin5 = 5,
        Pin6 = 6,
        Pin7 = 7,
        Pin8 = 8,
        Pin9 = 9,
        Pin10 = 10,
        Pin11 = 11,
        Pin12 = 12,
        Pin13 = 13,
        Pin14 = 14,
        Pin15 = 15,
    }

    export enum LEDNum {
        LED0 = 0,
        LED1 = 1,
        LED2 = 2,
        LED3 = 3,
        LED4 = 4,
        LED5 = 5,
        LED6 = 6,
        LED7 = 7,
        LED8 = 8,
        LED9 = 9,
        LED10 = 10,
        LED11 = 11,
        LED12 = 12,
        LED13 = 13,
        LED14 = 14,
        LED15 = 15,
    }

    export enum PWMNum {
        PWM8 = 8,
        PWM9 = 9,
        PWM10 = 10,
        PWM11 = 11,
        PWM12 = 12,
        PWM13 = 13,
        PWM14 = 14,
        PWM15 = 15,
    }

    export enum Motor {
        MotorLF = 1,    //FRONT
        MotorRF = 0,
        MotorLR = 2,    //REAR
        MotorRR = 3,
    }

    export enum MoveDir {
        deg0 = 0,
        deg45 = 45,
        deg90 = 90,
        deg135 = 135,
        deg180 = 180,
        deg225 = 225,
        deg270 = 270,
        deg315 = 315,
    }


    const pinNumber = -1
    const minOffset = 5
    const midOffset = 15
    const maxOffset = 25
    const position = 90

    function calcFreqPrescaler(freq: number): number {
        return (25000000 / (freq * chipResolution)) - 1;
    }


    function write(chipAddress: number, register: number, value: number): void {
        const buffer = pins.createBuffer(2)
        buffer[0] = register
        buffer[1] = value
        pins.i2cWriteBuffer(chipAddress, buffer, false)
    }

    /**
         * Used to setup the chip, will cause the chip to do a full reset and turn off all outputs.
         * @param chipAddress [64-125] The I2C address of your PCA9685; eg: 64
         * @param freq [40-1000] Frequency (40-1000) in hertz to run the clock cycle at; eg: 50
         */
    //% block="Init addr =$chipAddress, freq =$newFreq"
    export function init(chipAddress: number = 0x40, newFreq: number = 50) {
        const buf = pins.createBuffer(2)
        freq = (newFreq > 1000 ? 1000 : (newFreq < 40 ? 40 : newFreq))
        const prescaler = calcFreqPrescaler(freq)
        CHIP_ADDR = chipAddress
        write(chipAddress, modeRegister1, sleep)

        write(chipAddress, PrescaleReg, prescaler)

        write(chipAddress, allChannelsOnStepLowByte, 0x00)
        write(chipAddress, allChannelsOnStepHighByte, 0x00)
        write(chipAddress, allChannelsOffStepLowByte, 0x00)
        write(chipAddress, allChannelsOffStepHighByte, 0x00)

        write(chipAddress, modeRegister1, wake)

        control.waitMicros(1000)
        write(chipAddress, modeRegister1, restart)
    }

    function calcFreqOffset(freq: number, offset: number) {
        return ((offset * 1000) / (1000 / freq) * chipResolution) / 10000
    }

    /**
     * Used to set the pulse range (0-4095) of a given pin on the PCA9685
     * @param chipAddress [64-125] The I2C address of your PCA9685; eg: 64
     * @param pinNumber The pin number (0-15) to set the pulse range on
     * @param onStep The range offset (0-4095) to turn the signal on
     * @param offStep The range offset (0-4095) to turn the signal off
     */
    function setPinPulseRange(pinNumber: PinNum = 0, onStep: number = 0, offStep: number = 2048, chipAddress: number = 0x40): void {
        pinNumber = Math.max(0, Math.min(15, pinNumber))
        const buffer = pins.createBuffer(2)
        const pinOffset = PinRegDistance * pinNumber
        onStep = Math.max(0, Math.min(4095, onStep))
        offStep = Math.max(0, Math.min(4095, offStep))

        // Low byte of onStep
        write(chipAddress, pinOffset + channel0OnStepLowByte, onStep & 0xFF)

        // High byte of onStep
        write(chipAddress, pinOffset + channel0OnStepHighByte, (onStep >> 8) & 0x0F)

        // Low byte of offStep
        write(chipAddress, pinOffset + channel0OffStepLowByte, offStep & 0xFF)

        // High byte of offStep
        write(chipAddress, pinOffset + channel0OffStepHighByte, (offStep >> 8) & 0x0F)
    }

    /**
     * Used to set the duty cycle (0-100) of a given led connected to the PCA9685
     * @param ledNumber The number (0-15) of the LED to set the duty cycle on
     * @param dutyCycle The duty cycle (0-100) to set the LED to
     */
    //% block
    //% subcategory=Servo/Motor
    export function setLedDutyCycle(ledNum: LEDNum = 0, dutyCycle: number): void {
        ledNum = Math.max(0, Math.min(15, ledNum))
        dutyCycle = Math.max(0, Math.min(100, dutyCycle))
        const pwm = (dutyCycle * (chipResolution - 1)) / 100
        return setPinPulseRange(<number>ledNum, 0, pwm, CHIP_ADDR)
    }

    function degrees180ToPWM(freq: number, degrees: number, offsetStart: number, offsetEnd: number): number {
        // Calculate the offset of the off point in the freq
        offsetEnd = calcFreqOffset(freq, offsetEnd)
        offsetStart = calcFreqOffset(freq, offsetStart)
        const spread: number = offsetEnd - offsetStart
        const calcOffset: number = ((degrees * spread) / 180) + offsetStart
        // Clamp it to the bounds
        return Math.max(offsetStart, Math.min(offsetEnd, calcOffset))
    }

    /**
     * Used to move the given servo to the specified degrees (0-180) connected to the PCA9685
     * @param servoNum The number (1-16) of the servo to move
     * @param degrees The degrees (0-180) to move the servo to
     */
    //% block
    //% subcategory=Servo/Motor
    export function setServoPosition(servoNum: PWMNum = 8, degrees: number): void {
        servoNum = Math.max(0, Math.min(15, servoNum))
        degrees = Math.max(0, Math.min(180, degrees))
        const pwm = degrees180ToPWM(freq, degrees, minOffset, maxOffset)
        return setPinPulseRange(<number>servoNum, 0, pwm, CHIP_ADDR)
    }


    /**
     * Single Motor Control
     * @param speed [-100,100] percent of fullspeed, negative is reverse
     */
    //% block
    //% subcategory=Servo/Motor
    export function MotorControl(motor: Motor, speed: number = 0): void {
        speed = Math.max(-100, Math.min(100, speed))
        if (speed > 0) {
            setLedDutyCycle(2 * motor, 100 - Math.abs(speed))
            setLedDutyCycle(2 * motor + 1, 100)
        }
        else {
            setLedDutyCycle(2 * motor + 1, 100 - Math.abs(speed))
            setLedDutyCycle(2 * motor, 100)
        }
    }

    /**
     * Car translation use Mecanum wheel
     * @param speed [0,100] percent of fullspeed
     * @param degrees [0,360] direction of translation
     */
    //% block="Car Translation speed =$speed, degrees =$degrees"
    //% subcategory=Servo/Motor
    export function CarTranslation(speed: number = 0, degrees: MoveDir = MoveDir.deg0): void {
        speed = Math.max(0, Math.min(100, speed))
        degrees = Math.max(0, Math.min(360, degrees))
        const speed_lim = speed * Math.sin(Math.PI / 4)
        const rad = Math.PI * degrees / 180
        const vx = speed_lim * Math.cos(rad)
        const vy = -speed_lim * Math.sin(rad)
        MotorControl(Motor.MotorLF, vx - vy)
        MotorControl(Motor.MotorRF, vx + vy)
        MotorControl(Motor.MotorLR, vx + vy)
        MotorControl(Motor.MotorRR, vx - vy)
    }

    /**
     * Car Rotation use Mecanum wheel
     * @param speed [-100,100] percent of fullspeed
     */
    //% block="Car Rotation speed =$speed"
    //% subcategory=Servo/Motor
    export function CarRotation(speed: number = 0): void {
        speed = Math.max(-100, Math.min(100, speed))
        MotorControl(Motor.MotorLF, speed)
        MotorControl(Motor.MotorRF, -speed)
        MotorControl(Motor.MotorLR, speed)
        MotorControl(Motor.MotorRR, -speed)
    }

    export enum startbit_lineFollower {
        //% blockId="S1_OUT_S2_OUT" block="Sensor1 and sensor2 are out black line"
        S1_OUT_S2_OUT = 0x00,
        //% blockId="S1_OUT_S2_IN" block="Sensor2 in black line but sensor1 not"
        S1_OUT_S2_IN = 0x01,
        //% blockId="S1_IN_S2_OUT" block="Sensor1 in black line but sensor2 not"
        S1_IN_S2_OUT = 0x02,
        //% blockId="S1_IN_S2_IN" block="Sensor1 and sensor2 are in black line "
        S1_IN_S2_IN = 0x03
    }

    export enum startbit_lineFollowPort {
        //% block="Port 1"
        port1 = 0x01
    }

    let lineFollowPin1: AnalogPin;
    let lineFollowPin2: AnalogPin;
    //% weight=92 blockId=lineFollowSensor_init  block="Initialize lineFollowSensor|port %port"
    //% subcategory=lineFollowSensor
    export function lineFollowSensor_init(port: startbit_lineFollowPort) {
        switch (port) {
            case startbit_lineFollowPort.port1:
                lineFollowPin1 = AnalogPin.P1;
                lineFollowPin2 = AnalogPin.P2;
                break;
        }
    }

    export enum startbit_LineFollowerSensor {
        //% block="Sensor 1"
        LFSensor_1 = 0x00,
        //% block="Sensor 2"
        LFSensor_2 = 0x01
    }

    export enum startbit_iic {
        //% block="Port 3"
        port3 = 0x03,
        //% block="Port 4"
        port4 = 0x04,
        //% block="Port 6"
        port6 = 0x06
    }

    //% weight=86 blockId=lineFollow_iic_init  block="Initialize lineFollow iic|port %port"
    //% subcategory=lineFollowSensor
    export function lineFollow_iic_init(port: startbit_iic) {
        switch (port) {
            case startbit_iic.port3:
                break;
            case startbit_iic.port4:
                break;
            case startbit_iic.port6:
                break;
        }
    }

    /**
* Get the condition of the line follower sensor
*/
    //% weight=96 blockId=startbit_readLineFollowerStatus block="Line follower status|%status"
    //% subcategory=lineFollowSensor
    export function startbit_readLineFollowerStatus(status: startbit_lineFollower): boolean {
        let s1 = 0;
        let s2 = 0;

        s1 = pins.analogReadPin(lineFollowPin1);
        s2 = pins.analogReadPin(lineFollowPin2);
        s1 = s1 * 255 / 1023;
        s2 = s2 * 255 / 1023;
        if (s1 < 200)
            s1 = 0;
        else
            s1 = 1;
        if (s2 < 200)
            s2 = 0;
        else
            s2 = 1;

        let s = ((1 & s1) << 1) | s2;
        if (s == status) {
            return true;
        }
        else {
            return false;
        }
    }

    /**
     * Get the line follower sensor port ad value
     */
    //% weight=89 blockId=startbit_lineSensorValue blockGap=50 block="Get line follower sensor|%sensor|ad value"
    //% subcategory=lineFollowSensor
    export function startbit_lineSensorValue(sensor: startbit_LineFollowerSensor): number {
        let s1 = 0;
        let s2 = 0;

        s1 = pins.analogReadPin(lineFollowPin1);
        s2 = pins.analogReadPin(lineFollowPin2);
        s1 = s1 * 255 / 1023;
        s2 = s2 * 255 / 1023;

        if (sensor == startbit_LineFollowerSensor.LFSensor_1) {
            return 255 - s1;
        }
        else {
            return 255 - s2;
        }

    }

    export enum startbit_LineFollowerSensors {
        //% block="S1"
        S1,
        //% block="S2"
        S2,
        //% block="S3"
        S3,
        //% block="S4"
        S4
    }

    export enum startbit_LineColor {
        //% block="Black"
        Black,
        //% block="White"
        White
    }

    const LINE_FOLLOWER_I2C_ADDR = 0x78

    //% weight=95 blockId=startbit_line_followers blockGap=50 block="Line follower %lineFollowerSensor in %LineColor ?"
    //% inlineInputMode=inline
    //% subcategory=lineFollowSensor
    export function startbit_line_followers(lineFollowerSensor: startbit_LineFollowerSensors, LineColor: startbit_LineColor): boolean {
        pins.i2cWriteNumber(LINE_FOLLOWER_I2C_ADDR, 1, NumberFormat.UInt8BE);
        let data = pins.i2cReadNumber(LINE_FOLLOWER_I2C_ADDR, NumberFormat.UInt8BE);
        let status = false;
        switch (lineFollowerSensor) {
            case startbit_LineFollowerSensors.S1:
                if (data & 0x01) {
                    if (LineColor == startbit_LineColor.Black) {
                        status = true;
                    }
                }
                else {
                    if (LineColor == startbit_LineColor.White) {
                        status = true;
                    }
                }
                break;

            case startbit_LineFollowerSensors.S2:
                if (data & 0x02) {
                    if (LineColor == startbit_LineColor.Black) {
                        status = true;
                    }
                }
                else {
                    if (LineColor == startbit_LineColor.White) {
                        status = true;
                    }
                }
                break;

            case startbit_LineFollowerSensors.S3:
                if (data & 0x04) {
                    if (LineColor == startbit_LineColor.Black) {
                        status = true;
                    }
                }
                else {
                    if (LineColor == startbit_LineColor.White) {
                        status = true;
                    }
                }
                break;

            case startbit_LineFollowerSensors.S4:
                if (data & 0x08) {
                    if (LineColor == startbit_LineColor.Black) {
                        status = true;
                    }
                }
                else {
                    if (LineColor == startbit_LineColor.White) {
                        status = true;
                    }
                }
                break;
        }
        return status;
    }


    export enum Functions {
        //% block="NoFunction"
        NoFunction,
        //% block="Facial recognition"
        FaceDetect = 1,
        //% block="Object detection"
        ObjectDetect,
        //% block="Classification"
        Classification,
        //% block="FeatureLearning"
        FeatureLearning,
        //% block="ColorDetect"
        ColorDetect,
        //% block="LineFollowing"
        LineFollowing,
        //% block="AprilTag"
        AprilTag,
        //% block="QrcodeScan"
        QrcodeScan,
        //% block="BarcodeScan"
        BarcodeScan,
        //% block="Number Recognition"
        NumberRecognition,
        //% block="Landmark Recognition"
        LandmarkRecognition,
    }

    export enum Landmarks {
        //% bock="NoLandmark"
        NoLandmark,
        //% block="Go forward"
        GoForward = 1,
        //% block="Turn left"
        TurnLeft = 2,
        //% block="Turn right"
        TurnRight = 3,
        //% block="Turn About"
        TurnAbout = 4,
        //% block="Stop"
        Stop = 5,
    }

    export enum Objects {
        //% block="Aeroplane"
        Aeroplane = 1,
        //% block="Bicycle"
        Bicycle,
        //% block="Bird"
        Bird,
        //% block="Boar"
        Boar,
        //% block="Bootle"
        Bootle,
        //% block="Bus"
        Bus,
        //% block="Car"
        Car,
        //% block="Cat"
        Cat,
        //% block="Chair"
        Chair,
        //% block="Cow"
        Cow,
        //% block="Diningtable"
        Diningtable,
        //% block="Dog"
        Dog,
        //% block="Horse"
        Horse,
        //% block="Motorbike"
        Motorbike,
        //% block="Person"
        Person,
        //% block="Pottedplant"
        Pottedplant,
        //% block="Sheep"
        Sheep,
        //% block="Sofa"
        Sofa,
        //% block="Train"
        Train,
        //% block="TvMonitorn"
        TvMonitorn
    }

    export enum Options {
        //% block="X"
        Pos_X = 0,
        //% block="Y"
        Pos_Y = 0x02,
        //% block="Width"
        Width = 0x04,
        //% block="Height"
        Height = 0x06
    }

    export enum Obj_Options {
        //% block="X"
        Pos_X = 0,
        //% block="Y"
        Pos_Y = 0x02,
        //% block="Width"
        Width = 0x04,
        //% block="Height"
        Height = 0x06,
        //% block="Confidence"
        Confidence = 0x08
    }

    export enum Line_Options {
        //% block="Start X"
        Start_X = 0x00,
        //% block="Start Y"
        Start_Y = 0x02,
        //% block="End X"
        END_X = 0x04,
        //% block="End Y"
        END_Y = 0x06,
        //% block="Angle"
        Angle = 0x08,
        //% block="Offset"
        Offset = 0x0A
    }

    export enum AprilTag_Options {
        //% block="Center X"
        Pos_X = 0x00,
        //% block="Center Y"
        Pos_Y = 0x02,
        //% block="W"
        Width = 0x04,
        //% block="H"
        Height = 0x06,
        //% block="X Translation"
        X_T = 0x08,
        //% block="X Rotation"
        X_R = 0x0A,
        //% block="Y Translation"
        Y_T = 0x0C,
        //% block="Y Rotation"
        Y_R = 0x0E,
        //% block="Z Translation"
        Z_T = 0x10,
        //% block="Z Rotation"
        Z_R = 0x12,
    }

    export enum LED_STATE {
        //% block="ON"
        ON = 1,
        //% block="OFF"
        OFF = 0
    }

    export enum DEV_ADDR {
        //% block="0x32"
        x32 = 0x32,
        //% block="0x21"
        x21 = 0x21,
        //% block="0x22"
        x22 = 0x22,
        //% block="0x23"
        x23 = 0x23,
        //% block="0x24"
        x24 = 0x24,
        //% block="0x31"
        x31 = 0x31,
        //% block="0x33"
        x33 = 0x33,
        //% block="0x34"
        x34 = 0x34,
        //% block="0x41"
        x41 = 0x41,
        //% block="0x42"
        x42 = 0x42,
        //% block="0x43"
        x43 = 0x43,
        //% block="0x44"
        x44 = 0x44,
        //% block="0x51"
        x51 = 0x45,
        //% block="0x52"
        x52 = 0x52,
        //% block="0x53"
        x53 = 0x53,
        //% block="0x54"
        x54 = 0x54,
    }

    let Current = Functions.NoFunction;
    let ResultBuf: Buffer;
    let WONDERCAM_I2C_ADDR = 0x32

    function i2cwrite(reg: number, value: number) {
        let buf = pins.createBuffer(3)
        buf.setNumber(NumberFormat.UInt8LE, 0, reg & 0xFF)
        buf.setNumber(NumberFormat.UInt8LE, 1, (reg >> 8) & 0xFF)
        buf.setNumber(NumberFormat.UInt8LE, 2, value & 0xFF)
        pins.i2cWriteBuffer(WONDERCAM_I2C_ADDR, buf)
    }

    function i2creadtobuf(reg: number, length: number): Buffer {
        let buf = pins.createBuffer(2)
        buf.setNumber(NumberFormat.UInt8LE, 0, reg & 0xFF)
        buf.setNumber(NumberFormat.UInt8LE, 1, (reg >> 8) & 0xFF)
        pins.i2cWriteBuffer(WONDERCAM_I2C_ADDR, buf)
        return pins.i2cReadBuffer(WONDERCAM_I2C_ADDR, length)
    }

    function i2creadnum(reg: number): number {
        let buf = i2creadtobuf(reg, 1)
        return buf.getNumber(NumberFormat.UInt8LE, 0)
    }

    /**
     * TODO:初始化I2C， 初始化WonderCam
     * @param dev_addr i2c address, eg: DEV_ADDR.x32
     */
    //% weight=180
    //% block="Initialize WonderCam at |$dev_addr|"
    export function wondercam_init(dev_addr = DEV_ADDR.x32): void {
        WONDERCAM_I2C_ADDR = dev_addr
        while (i2creadnum(0) != 'v'.charCodeAt(0)) {
            basic.showString("E")
        }
        basic.clearScreen()
    }

    /**
     * TODO: 获取WonderCam正在运行的功能，返回当前运行功能的序号
     */
    //% weight=145
    //% block="The running function"
    export function CurrentFunc(): Functions {
        return i2creadnum(0x0035)
    }

    /**
     * TODO: 判断当前运行的功能是否是某个功能
     */
    //% weight=149
    //% block="Is the running function |$func|?"
    //% func.defl=Functions.FaceDetect
    export function CurrentFuncIs(func: Functions): boolean {
        if (i2creadnum(0x0035) == func) {
            return true
        }
        return false
    }

    /**
     * TODO: 获取不同功能对应的功能序号
     */
    //% weight=1
    //% block="$func"
    //% func.defl=Functions.FaceDetect
    export function FunctoNum(func: Functions): number {
        return func
    }

    /**
     * TODO: 切换功能
     */
    //% weight=140
    //% block="Switch to $newfunc"
    //% newfunc.defl=Functions.FaceDetect
    export function ChangeFunc(newfunc: Functions): void {
        let count = 0;
        i2cwrite(0x0035, newfunc);
        basic.pause(100)
        while (true) {
            if (CurrentFuncIs(newfunc)) {
                break
            } else {
                if (count >= 80) {
                    break;
                }
                basic.pause(50)
                count++
            }
        }
    }

    /**
     * TODO: 开关LED
     */
    //% weight=100
    //% block="Turn |$newstate| led"
    //% newstate.defl=LED_STATE.ON
    export function TurnOnOrOffLed(newstate: LED_STATE): void {
        i2cwrite(0x0030, newstate);
    }

    /**
     * TODO: 设置LED亮度
     */
    //% weight=90
    //% block="Set led brightness as $newlevel"
    //% newlevel.defl=100 newlevel.min=0 newlevel.max=100
    export function SetLedBrightness(newlevel: number): void {
        i2cwrite(0x0031, newlevel);
    }

    /**
     * TODO: 更新WonderCam的处理结果
     */
    //% weight=120
    //% block="Update and get results"
    export function UpdateResult(): void {
        let func = CurrentFunc()
        switch (func) {
            case Functions.FaceDetect: //人脸识别 结果地址
                ResultBuf = i2creadtobuf(0x0400, 512)
                Current = Functions.FaceDetect;
                break;
            case Functions.ObjectDetect: //物品识别 结果地址
                ResultBuf = i2creadtobuf(0x0800, 512)
                Current = Functions.ObjectDetect
                break;
            case Functions.Classification: //图像分类 结果地址
                ResultBuf = i2creadtobuf(0x0C00, 128)
                Current = Functions.Classification
                break;
            case Functions.NumberRecognition: //图像分类 结果地址
                ResultBuf = i2creadtobuf(0x0D00, 128)
                Current = Functions.NumberRecognition
                break;
            case Functions.LandmarkRecognition: //图像分类 结果地址
                ResultBuf = i2creadtobuf(0x0D80, 128)
                Current = Functions.LandmarkRecognition
                break;
            case Functions.FeatureLearning:  //特征学习 结果地址
                ResultBuf = i2creadtobuf(0x0E00, 128)
                Current = Functions.FeatureLearning
                break;
            case Functions.ColorDetect: // 颜色识别 结果地址
                ResultBuf = i2creadtobuf(0x1000, 400)
                Current = Functions.ColorDetect
                break;
            case Functions.LineFollowing: //视觉巡线 结果地址
                ResultBuf = i2creadtobuf(0x1400, 256)
                Current = Functions.LineFollowing
                break;
            case Functions.AprilTag:
                ResultBuf = i2creadtobuf(0x1E00, 512);
                Current = Functions.AprilTag
                break;
            case Functions.QrcodeScan: //QRCODE 结果地址
                ResultBuf = i2creadtobuf(0x1800, 512)
                Current = Functions.QrcodeScan
                break;
            case Functions.BarcodeScan: //BAR CODE 结果地址
                ResultBuf = i2creadtobuf(0x1C00, 512)
                Current = Functions.BarcodeScan
                break;
            default:
                Current = Functions.NoFunction
                break;
        }
    }

    /**
     * TODO: 是否检测到了人脸
     */
    //% weight=160
    //% block="Is any face detected?"
    //% subcategory="Facial recognition"
    export function IsDetectFace(): boolean {
        if (Current == Functions.FaceDetect) {
            if (ResultBuf.getNumber(NumberFormat.UInt8LE, 1) > 0) {
                return true
            }
        }
        return false
    }

    /**
     * TODO: 获取识别到的人脸个数
     */
    //% weight=150
    //% block="Total number of detected faces"
    //% subcategory="Facial recognition"
    //% subcategory.loc.zh="人脸识别"
    export function FaceNum(): number {
        if (Current == Functions.FaceDetect) {
            return ResultBuf.getNumber(NumberFormat.UInt8LE, 1);
        }
        return 0;
    }

    /**
     * TODO: 是否识别到已经学习的人脸
     */
    //% weight=140
    //% block="Is any learned face recognized?"
    //% subcategory="Facial recognition"
    export function IsDetectedLearnedFace(): boolean {
        if (Current == Functions.FaceDetect) {
            if (ResultBuf.getNumber(NumberFormat.UInt8LE, 2) > 0) {
                return true;
            }
        }
        return false;
    }

    /**
     * TODO: 获取识别到的已经学习的人脸个数
     */
    //% weight=135
    //% block="Number of learned faces recognized"
    //% subcategory="Facial recognition"
    export function LearnedFaceNum(): number {
        if (Current == Functions.FaceDetect) {
            return ResultBuf.getNumber(NumberFormat.UInt8LE, 2);
        }
        return 0;
    }

    /**
     * TODO: 获取识别到的未学习的人脸个数
     */
    //% weight=130
    //% block="Is any unlearned face detected?"
    //% subcategory="Facial recognition"
    export function IsDetectUnLeanedFace(): boolean {
        if (Current == Functions.FaceDetect) {
            if (ResultBuf.getNumber(NumberFormat.UInt8LE, 3) > 0) {
                return true;
            }
        }
        return false;
    }

    /**
     * TODO: 获取识别到的未学习的人脸个数
     */
    //% weight=120
    //% block="Number of unlearned faces detected"
    //% subcategory="Facial recognition"
    export function UnLearnedFaceNum(): number {
        if (Current == Functions.FaceDetect) {
            return ResultBuf.getNumber(NumberFormat.UInt8LE, 3);
        }
        return 0;
    }

    /**
     * TODO: 是否识别到了指定ID的人脸
     * @param id[1-5] eg: 1
     */
    //% weight=110
    //% block="Is the face ID:$id recognized"
    //% subcategory="Facial recognition"
    export function IsDetectedFace(id: number): boolean {
        if (Current == Functions.FaceDetect) {
            for (let i = 4; i < 4 + 29; i++) {  // 逐个对比是否有这个id
                if (ResultBuf.getNumber(NumberFormat.UInt8LE, i) == id) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * TODO: 获返回指定ID的人脸的位置数据。若成功返回数据,失败返回0
     */
    //% weight=95
    //% block="$opt of face ID: $id"
    //% id.defl=1 id.min=1 id.max=5
    //% opt.defl=Options.Pos_X
    //% subcategory="Facial recognition"
    export function getlearnedFaceY(opt: Options, id: number): number {
        for (let i = 4; i < 4 + 29; i++) {  // 逐个对比是否有这个id
            if (ResultBuf.getNumber(NumberFormat.UInt8LE, i) == id) {
                let index = i - 4;
                return ResultBuf.getNumber(NumberFormat.Int16LE, (0x30 + opt) + index * 16)
            }
        }
        return 0;
    }

    /**
     * TODO: 获返回指定Index的未学习的人脸的位置数据。若成功返回数据,失败返回0
     */
    //% weight=13
    //% block="|$opt| of the no.|$index| unlearned face recognized"
    //% index.defl=1 index.min=1 index.max=20
    //% opt.defl=Options.Pos_X
    //% subcategory="Facial recognition"
    export function getUnlearnedFaceX(opt: Options, index: number): number {
        let num = 0;
        for (let i = 4; i < 4 + 29; i++) {  // 逐个对比是否有这个id
            if (ResultBuf.getNumber(NumberFormat.UInt8LE, i) == 0xFF) {
                num += 1
                if (num == index) {
                    return ResultBuf.getNumber(NumberFormat.Int16LE, (0x30 + opt) + (i - 4) * 16)
                }
            }
        }
        return 0;
    }

    /**
     * TODO: 是否识别到了物品
     */
    //% weight=100 block="Is any object detected?"
    //% subcategory="Object detection"
    export function IsDetectObject(): boolean {
        if (Current == Functions.ObjectDetect) {
            if (ResultBuf.getNumber(NumberFormat.UInt8LE, 1) > 0) {
                return true
            }
        }
        return false
    }

    /**
     * TODO: 获取识别到的物品总数
     */
    //% weight=97 blockId=ObjNum block="Total number of objects detected"
    //% subcategory="Object detection"
    export function ObjNum(): number {
        if (Current == Functions.ObjectDetect) {
            return ResultBuf.getNumber(NumberFormat.UInt8LE, 1);
        } else {
            return 0;
        }
    }

    /**
     * TODO: 是否识别到了指定ID的物品
     */
    //% weight=95 blockId=IsDetectedObject block="Is $id| detected?"
    //% subcategory="Object detection"
    export function IsDetectedObjectOfId(id: Objects): boolean {
        if (Current == Functions.ObjectDetect) {
            for (let i = 2; i < 2 + 29; i++) {  // 逐个对比是否有这个id
                if (ResultBuf.getNumber(NumberFormat.UInt8LE, i) == id) {
                    return true;
                }
            }
            return false;
        } else {
            return false;
        }
    }

    /**
     * TODO: 识别到的指定ID的物品的个数
     */
    //% weight=85  block="Number of |$id| detected"
    //% subcategory="Object detection"
    export function NumOfDetectedObject(id: Objects): number {
        let num = 0;
        if (Current == Functions.ObjectDetect) {
            for (let i = 2; i < 2 + 29; i++) {  // 逐个对比是否有这个id
                if (ResultBuf.getNumber(NumberFormat.UInt8LE, i) == id) {
                    num += 1;
                }
            }
        }
        return num;
    }

    /**
     * TODO: 获取识别到的指定物品的指定序号的结果的数据
     */
    //% weight=75 block="|$opt| of the no.|$index| |$id| detected"
    //% opt.defl=Obj_Options.Pos_X
    //% index.defl=1 index.min=1 index.max=10
    //% id.defl=Objects.Aeroplane
    //% subcategory="Object detection"
    export function getObjectW(opt: Obj_Options, index: number, id: Objects): number {
        let num = 0
        let addr = 0
        if (Current == Functions.ObjectDetect) {
            for (let i = 2; i < 2 + 29; i++) {  // 逐个对比是否有这个id
                if (ResultBuf.getNumber(NumberFormat.UInt8LE, i) == id) {
                    num += 1
                    if (num == index) {
                        return ResultBuf.getNumber(NumberFormat.UInt16LE, (0x30 + opt) + (i - 2) * 16)
                    }
                }
            }
        }
        return 0
    }

    //图像分类
    /**
     * TODO: 获取置信度最大的ID
     */
    //% weight=82 blockId=MaxConfidenceID block="The most confident ID"
    //% id.defl=1 id.min=1 id.max=20
    //% subcategory="Classification"
    export function MaxConfidenceID(): number {
        if (Current == Functions.Classification) {
            return ResultBuf.getNumber(NumberFormat.UInt8LE, 0x01);
        }
        return 0
    }

    /**
     * TODO: 获取最大的置信度
     */
    //% weight=81 blockId=MaxConfidence block="The most confident"
    //% id.defl=1 id.min=1 id.max=20
    //% subcategory="Classification"
    export function MaxConfidence(): number {
        if (Current == Functions.Classification) {
            let c = ResultBuf.getNumber(NumberFormat.UInt16LE, 0x02);
            return (c / 10000.0)
        }
        return 0
    }

    /**
     * TODO: 获取指定ID的的置信度
     */
    //% weight=80 blockId=ConfidenceOfId block="Confident of ID:$id"
    //% id.defl=1 id.min=1 id.max=20
    //% subcategory="Classification"
    export function ConfidenceOfIdClassification(id: number): number {
        if (Current == Functions.Classification) {
            let c = ResultBuf.getNumber(NumberFormat.UInt16LE, 0x10 + ((id - 1) * 4))
            return (c / 10000.0)
        }
        return 0
    }

    //数字识别
    /**
     * TODO: 获取置信度最大的数字
     */
    //% weight=82 blockId=NumberWithMaxConfidence block="The most confident Number"
    //% subcategory="Number recognition"
    export function NumberWithMaxConfidence(): number {
        if (Current == Functions.NumberRecognition) {
            return ResultBuf.getNumber(NumberFormat.UInt8LE, 0x01);
        }
        return 0
    }

    /**
     * TODO: 获取数字识别最大的置信度
     */
    //% weight=81 blockId=MaxConfidenceOfNumber block="The most confident"
    //% id.defl=1 id.min=1 id.max=5
    //% subcategory="Number recognition"
    export function MaxConfidenceOfNumber(): number {
        if (Current == Functions.NumberRecognition) {
            let c = ResultBuf.getNumber(NumberFormat.UInt16LE, 0x02);
            return (c / 10000.0)
        }
        return 0
    }

    /**
     * TODO: 获取指定数字的的置信度
     */
    //% weight=80 blockId=ConfidenceOfNumber block="Confident of Number:$id"
    //% id.defl=1 id.min=1 id.max=5
    //% subcategory="Number recognition"
    export function ConfidenceOfNumber(id: number): number {
        if (Current == Functions.NumberRecognition) {
            let c = ResultBuf.getNumber(NumberFormat.UInt16LE, 0x10 + ((id - 1) * 4))
            return (c / 10000.0)
        }
        return 0
    }

    //路标识别
    /**
     * TODO: 获取置信度最大的路标
     */
    //% weight=82 blockId=LandmarkWithMaxConfidence block="The most confident Number"
    //% subcategory="Landmark recognition"
    export function LandmarkWithMaxConfidence(): Landmarks {
        if (Current == Functions.LandmarkRecognition) {
            return ResultBuf.getNumber(NumberFormat.UInt8LE, 0x01);
        }
        return 0
    }

    /**
     * TODO: 获取路标识别最大的置信度
     */
    //% weight=81 blockId=MaxConfidenceOfLandmark block="The most confident"
    //% subcategory="Landmark recognition"
    export function MaxConfidenceOfLandmark(): number {
        if (Current == Functions.LandmarkRecognition) {
            let c = ResultBuf.getNumber(NumberFormat.UInt16LE, 0x02);
            return (c / 10000.0)
        }
        return 0
    }

    /**
     * TODO: 获取指路标的的置信度
     */
    //% weight=80 blockId=ConfidenceOfLandmark block="Confident of Number:$id"
    //% id.defl=1 id.min=1 id.max=5
    //% subcategory="Landmark recognition"
    export function ConfidenceOfLandmark(id: Landmarks): number {
        if (Current == Functions.LandmarkRecognition) {
            let c = ResultBuf.getNumber(NumberFormat.UInt16LE, 0x10 + ((id - 1) * 4))
            return (c / 10000.0)
        }
        return 0
    }
    //% weight=60 blockId=GetLandmarkObj block="|$in_|"
    //% subcategory="Landmark recognition"
    export function LandmarkObj(in_: Landmarks): Landmarks {
        return in_
    }

    //特征学习
    /**
     * TODO: 获取置信度最大的ID
     */
    //% weight=82 block="The most confident ID"
    //% id.defl=1 id.min=1 id.max=20
    //% subcategory="FeatureLearning"
    export function FlMaxConfidenceID(): number {
        if (Current == Functions.FeatureLearning) {
            return ResultBuf.getNumber(NumberFormat.UInt8LE, 0x01);
        }
        return 0
    }

    /**
     * TODO: 获取最大的置信度
     */
    //% weight=81 block="The most confident"
    //% id.defl=1 id.min=1 id.max=20
    //% subcategory="FeatureLearning"
    export function FlMaxConfidence(): number {
        if (Current == Functions.FeatureLearning) {
            let c = ResultBuf.getNumber(NumberFormat.UInt16LE, 0x02);
            return (c / 10000.0)
        }
        return 0
    }

    /**
     * TODO: 获取指定ID的的置信度
     */
    //% weight=80 block="Confident of ID:$id"
    //% id.defl=1 id.min=1 id.max=7
    //% subcategory="FeatureLearning"
    export function FlConfidenceOfId(id: number): number {
        if (Current == Functions.FeatureLearning) {
            let c = ResultBuf.getNumber(NumberFormat.UInt16LE, 0x10 + ((id - 1) * 4))
            return (c / 10000.0)
        }
        return 0
    }

    //颜色识别
    /**
     * TODO: 是否识别到了色块
     */
    //% weight=99 block="Is color detected?"
    //% subcategory="Color detection"
    export function IsDetectedColorblobs(): boolean {
        if (Current == Functions.ColorDetect) {
            if (ResultBuf.getNumber(NumberFormat.UInt8LE, 0x01) > 0) {
                return true;
            }
        }
        return false
    }

    /**
     * TODO: 识别到的色块总数
     */
    //% weight=90 block="Total number of detected colors"
    //% subcategory="Color detection"
    export function NumberOfColorblobs(): number {
        if (Current == Functions.ColorDetect) {
            return ResultBuf.getNumber(NumberFormat.UInt8LE, 0x01);
        }
        return 0
    }

    /**
     * TODO: 是否识别到了指定ID的颜色
     */
    //% weight=80 block="Is color ID:$id detected"
    //% id.defl=1 id.min=1 id.max=7
    //% subcategory="Color detection"
    export function isDetectedColorId(id: number): boolean {
        let num = NumberOfColorblobs()
        if (Current == Functions.ColorDetect) {
            for (let i = 2; i < 2 + num; i++) {  // 逐个对比是否有这个id
                if (ResultBuf.getNumber(NumberFormat.UInt8LE, i) == id) {
                    return true;
                }
            }
        }
        return false
    }

    /**
     * TODO: 返回指定ID颜色的位置数据
     */
    //% weight=75 block="|$opt| of color ID:|$id| detected"
    //% id.defl=1 id.min=1 id.max=7
    //% opt.defl=Options.Pos_X
    //% subcategory="Color detection"
    export function XOfColorId(opt: Options, id: number): number {
        let num = NumberOfColorblobs()
        if (Current == Functions.ColorDetect) {
            for (let i = 2; i < 2 + num; i++) {  // 逐个对比是否有这个id
                if (ResultBuf.getNumber(NumberFormat.UInt8LE, i) == id) {
                    return ResultBuf.getNumber(NumberFormat.Int16LE, (0x30 + opt) + ((i - 2) * 16));
                }
            }
        }
        return 0
    }

    //视觉巡线
    /**
     * TODO: 是否识别到了线
     */
    //% weight=100 block="Is any line detected?"
    //% subcategory="LineFollowing"
    export function isDetectedLine(): boolean {
        let num = NumberOfLines()
        if (Current == Functions.LineFollowing) {
            if (ResultBuf.getNumber(NumberFormat.UInt8LE, 0x01) > 0) {
                return true;
            }
        }
        return false
    }

    /**
     * TODO: 识别到的线总数
     */
    //% weight=90 block="Total number of lines detected"
    //% subcategory="LineFollowing"
    export function NumberOfLines(): number {
        if (Current == Functions.LineFollowing) {
            return ResultBuf.getNumber(NumberFormat.UInt8LE, 0x01);
        }
        return 0
    }

    /**
     * TODO: 是否识别到了指定ID的线
     */
    //% weight=85 block="Is line ID:$id detected?"
    //% id.defl=1 id.min=1 id.max=3
    //% subcategory="LineFollowing"
    export function isDetectedLineId(id: number): boolean {
        let num = NumberOfLines()
        if (Current == Functions.LineFollowing) {
            for (let i = 2; i < 2 + num; i++) {  // 逐个对比是否有这个id
                if (ResultBuf.getNumber(NumberFormat.UInt8LE, i) == id) {
                    return true;
                }
            }
        }
        return false
    }

    /**
     * TODO: 返回指定ID的线的位置数据
     */
    //% weight=80 block="|$opt| of line ID:|$id|"
    //% id.defl=1 id.min=1 id.max=3
    //% opt.defl=Line_Options
    //% subcategory="LineFollowing"
    export function StartXOfLineId(opt: Line_Options, id: number): number {
        let num = NumberOfLines()
        if (Current == Functions.LineFollowing) {
            for (let i = 2; i < 2 + num; i++) {  // 逐个对比是否有这个id
                if (ResultBuf.getNumber(NumberFormat.UInt8LE, i) == id) {
                    let tmp = ResultBuf.getNumber(NumberFormat.Int16LE, (0x30 + opt) + ((i - 2) * 16))
                    switch (opt) {
                        case Line_Options.Angle:
                            if (tmp > 90) {
                                return tmp - 180
                            } else {
                                return tmp
                            }
                        case Line_Options.Offset:
                            tmp = Math.abs(tmp)
                            return tmp - 160
                        default:
                            return tmp
                    }

                }
            }
        }
        return 0
    }

    //AprilTag
    /**
     * TODO: 是否识别到了标签
     */
    //% weight=99 block="Is any Tag detected?"
    //% subcategory="AprilTag"
    export function isDetectedAprilTag(): boolean {
        if (Current == Functions.AprilTag) {
            if (ResultBuf.getNumber(NumberFormat.Int8LE, 0x01) > 0) {
                return true
            }
        }
        return false
    }

    /**
     * TODO: 识别到的全部标签个数
     */
    //% weight=90 block="Number of all tags detected "
    //% subcategory="AprilTag"
    export function numberAllTagDetected(): number {
        if (Current == Functions.AprilTag) {
            return ResultBuf.getNumber(NumberFormat.Int8LE, 0x01)
        }
        return 0
    }

    /**
     * TODO: 是否识别到了指定ID的标签
     */
    //% weight=80 block="Is tag ID:$id detected?"
    //% id.defl=1
    //% subcategory="AprilTag"
    export function isDetecteAprilTagId(id: number): boolean {
        if (Current == Functions.AprilTag) {
            let num = ResultBuf.getNumber(NumberFormat.Int8LE, 0x01)
            for (let i = 2; i < 2 + num; i++) {  // 逐个对比是否有这个id
                if (ResultBuf.getNumber(NumberFormat.UInt8LE, i) == id) {
                    return true;
                }
            }
        }
        return false
    }

    /**
     * TODO: 识别到的指定ID标签个数
     */
    //% weight=70 block="Number of tag ID:|$id| detected "
    //% subcategory="AprilTag"
    export function numTagIdDetected(id: number): number {
        let count = 0
        if (Current == Functions.AprilTag) {
            let num = ResultBuf.getNumber(NumberFormat.Int8LE, 0x01)
            for (let i = 2; i < 2 + num; i++) {  // 逐个对比是否有这个id
                if (ResultBuf.getNumber(NumberFormat.UInt8LE, i) == id) {
                    count += 1
                }
            }
        }
        return count
    }


    /**
     * TODO: 返回指定标签的位置数据
     */
    //% weight=60 block="|$opt| of No.|$index| Tag ID:|$id|"
    //% index.defl=1 index.min=1
    //% id.defl=0
    //% subcategory="AprilTag"
    export function getTagDataId(opt: AprilTag_Options, index: number, id: number): number {
        if (Current == Functions.AprilTag) {
            let num = ResultBuf.getNumber(NumberFormat.Int8LE, 0x01)
            for (let i = 2; i < 2 + num; i++) {  // 逐个对比是否有这个id
                if (ResultBuf.getNumber(NumberFormat.UInt8LE, i) == id) {
                    index -= 1
                    if (index == 0) {
                        switch (opt) {
                            case AprilTag_Options.Pos_X:
                                return ResultBuf.getNumber(NumberFormat.Int16LE, 0x30 + (32 * (i - 2)));
                            case AprilTag_Options.Pos_Y:
                                return ResultBuf.getNumber(NumberFormat.Int16LE, 0x30 + 2 + (32 * (i - 2)));
                            case AprilTag_Options.Width:
                                return ResultBuf.getNumber(NumberFormat.UInt16LE, 0x30 + 4 + (32 * (i - 2)));
                            case AprilTag_Options.Height:
                                return ResultBuf.getNumber(NumberFormat.UInt16LE, 0x30 + 6 + (32 * (i - 2)));
                            case AprilTag_Options.X_T:
                                return ResultBuf.getNumber(NumberFormat.Float32LE, 0x38 + (32 * (i - 2)));
                            case AprilTag_Options.X_R:
                                return ResultBuf.getNumber(NumberFormat.Float32LE, 0x38 + 4 + (32 * (i - 2)));
                            case AprilTag_Options.Y_T:
                                return ResultBuf.getNumber(NumberFormat.Float32LE, 0x38 + 8 + (32 * (i - 2)));
                            case AprilTag_Options.Y_R:
                                return ResultBuf.getNumber(NumberFormat.Float32LE, 0x38 + 12 + (32 * (i - 2)));
                            case AprilTag_Options.Z_T:
                                return ResultBuf.getNumber(NumberFormat.Float32LE, 0x38 + 16 + (32 * (i - 2)));
                            case AprilTag_Options.Z_R:
                                return ResultBuf.getNumber(NumberFormat.Float32LE, 0x38 + 20 + (32 * (i - 2)));
                        }
                    } else {
                        if (index <= 0) {
                            return 0;
                        }
                    }
                }
            }
        }
        return 0
    }

    //QrCode
    /**
     * TODO: 是否识别到了二维码
     */
    //% weight=100 block="Is any QRcode detected?"
    //% subcategory="QRcode scanning"
    export function isDetectedQrCode(): boolean {
        if (Current == Functions.QrcodeScan) {
            if (ResultBuf.getNumber(NumberFormat.Int8LE, 0x01) > 0) {
                return true;
            }
        }
        return false
    }

    /**
     * TODO: 是否识别到了指定ID的二维码
     */
    //% weight=90 block="是否识别到了ID:$id二维码"
    //% id.defl=1 id.min=1 id.max=5
    //% subcategory="QRcode scanning"
    // export function isDetecteQrCodeId(id:number): boolean {
    // let num = NumberOfLines()
    // if(Current == Functions.QrcodeScan){
    // for(let i = 2; i < num; i++){  // 逐个对比是否有这个id
    // if(ResultBuf.getNumber(NumberFormat.UInt8LE, i) == id){
    // return true;
    // }
    // }
    // }
    // return false
    // }
    /**
     * TODO: 识别到的二维码的数据长度
     */
    //% weight=80 block="Data length of detected QRcode"
    //% subcategory="QRcode scanning"
    export function LengthOfQrCodeData(): number {
        if (Current == Functions.QrcodeScan) {
            return ResultBuf.getNumber(NumberFormat.UInt16LE, 0x20)
        }
        return 0
    }

    /**
     * TODO: 以字符串形式返回识别到的二维码的数据
     */
    //% weight=70 block="String from the qrcode detected"
    //% subcategory="QRcode scanning"
    export function StringFromQrCodeData(): string {
        if (Current == Functions.QrcodeScan) {
            return ResultBuf.slice(0x30, LengthOfQrCodeData()).toString()
        }
        return ""
    }

    /**
     * TODO: 以数组形式返回识别到的二维码的数据
     */
    //% weight=60 block="Array from the qrcode detected"
    //% subcategory="QRcode scanning"
    export function ArrayFromQrCodeData(): Array<number> {
        if (Current == Functions.QrcodeScan) {
            return ResultBuf.slice(0x30, LengthOfQrCodeData()).toArray(NumberFormat.UInt8LE)
        }
        return []
    }

    //
    //BarCode 
    /**
     * TODO: 是否识别到了条形码
     */
    //% weight=100 block="Is a Barcode detected?"
    //% subcategory="Barcode scanning"
    export function isDetectedBarCode(): boolean {
        let num = NumberOfLines()
        if (Current == Functions.BarcodeScan) {
            if (ResultBuf.getNumber(NumberFormat.Int8LE, 0x01) > 0) {
                return true
            }
        }
        return false
    }

    /**
     * TODO: 是否识别到了指定ID的条形码
     */
    //% weight=90 block="是否识别到了ID:$id条形码"
    //% id.defl=1 id.min=1 id.max=5
    //% subcategory="Barcode scanning"
    // export function isDetectedBarCodeId(id:number): boolean {
    //     let num = NumberOfLines()
    //     if(Current == Functions.BarcodeScan){
    //         for(let i = 2; i < num; i++){  // 逐个对比是否有这个id
    //             if(ResultBuf.getNumber(NumberFormat.UInt8LE, i) == id){
    //                 return true;
    //             }
    //         }
    //     }
    //     return false
    // }
    /**
     * TODO: 识别到的条形码的数据长度
     */
    //% weight=80 block="Data length of detected Barcode"
    //% subcategory="Barcode scanning"
    export function LengthOfBarCodeData(): number {
        if (Current == Functions.BarcodeScan) {
            return ResultBuf.getNumber(NumberFormat.UInt16LE, 0x20)
        }
        return 0
    }

    /**
     * TODO: 以字符串形式返回识别到的条形码的数据
     */
    //% weight=70 block="String from the barcode detected"
    //% subcategory="Barcode scanning"
    export function StringFromBarCodeData(): string {
        if (Current == Functions.BarcodeScan) {
            return ResultBuf.slice(0x30, LengthOfBarCodeData()).toString()
        }
        return ""
    }

    /**
     * TODO: 以数组形式返回识别到的条形码的数据
     */
    //% weight=60 block="Array from the barcode detected"
    //% subcategory="Barcode scanning"
    export function ArrayFromBarrCodeData(): Array<number> {
        if (Current == Functions.BarcodeScan) {
            return ResultBuf.slice(0x30, LengthOfBarCodeData()).toArray(NumberFormat.UInt8LE)
        }
        return []
    }
}
