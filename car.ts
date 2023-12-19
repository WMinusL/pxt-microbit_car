/**
 * microbit_car
 */
//% weight=100 color=#0fbc11 icon="\uf021"
namespace microbit_car {
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
        deg0 =  0,  
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
    export function MotorControl(motor: Motor,speed: number = 0): void {
        speed = Math.max(-100, Math.min(100, speed))
        if(speed>0)
        {
            setLedDutyCycle(2 * motor, 100 - Math.abs(speed))
            setLedDutyCycle(2 * motor + 1, 100)
        }
        else
        {
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
        const rad = Math.PI*degrees/180
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
        speed = Math.max(0, Math.min(100, speed))
        MotorControl(Motor.MotorLF,  speed)
        MotorControl(Motor.MotorRF, -speed)
        MotorControl(Motor.MotorLR,  speed)
        MotorControl(Motor.MotorRR, -speed)
    }

}