// micro:bit op de micro:Maqueen V4.x - draadloos via Bluetooth, voor ROS 2
//
// Compileren en flashen (micro:bit via USB):
//     powershell -ExecutionPolicy Bypass -File .\flash.ps1
//
// Protocol:
//   PC -> micro:bit :  "M <links> <rechts>\n"            snelheden -255..255
//   micro:bit -> PC :  "S <lijnL> <lijnR> <afstand_cm> <kompas_graden>\n"  ~10x/s
// Veiligheid: 0,5 s geen commando of Bluetooth weg = motoren stoppen.
//
// KOMPAS: de eerste keer vraagt de micro:bit om te kalibreren
// ("TILT TO FILL SCREEN"). Doe dat met de micro:bit OP de robot en de
// batterijen erin: kantel de hele robot tot alle lampjes branden.

// KNOPPEN op de micro:bit:
//   A      = NOODSTOP (motoren uit tot je B, A+B of start op de pc gebruikt)
//   B      = verkennen starten
//   A + B  = terug naar huis

const MAQUEEN = 0x10
let laatsteCmd = 0
let verbonden = false
let noodstop = false

function motor(reg: number, snelheid: number) {
    snelheid = Math.constrain(snelheid, -255, 255)
    let buf = pins.createBuffer(3)
    buf[0] = reg
    buf[1] = snelheid >= 0 ? 0 : 1
    buf[2] = Math.abs(snelheid)
    pins.i2cWriteBuffer(MAQUEEN, buf)
}

function motors(links: number, rechts: number) {
    motor(0x00, links)   // M1 = links
    motor(0x02, rechts)  // M2 = rechts
}

function afstandCm(): number {
    pins.digitalWritePin(DigitalPin.P1, 0)
    control.waitMicros(2)
    pins.digitalWritePin(DigitalPin.P1, 1)
    control.waitMicros(10)
    pins.digitalWritePin(DigitalPin.P1, 0)
    let t = pins.pulseIn(DigitalPin.P2, PulseValue.High, 30000)
    if (t <= 0) {
        return -1
    }
    return Math.round(t / 58 * 10) / 10
}

motors(0, 0)
// EERST de UART-dienst starten, anders ziet de pc hem niet
bluetooth.startUartService()

// Kompas uitlezen start de kalibratie als die nog niet gebeurd is
// ("TILT TO FILL SCREEN": robot kantelen tot alle lampjes branden)
input.compassHeading()
basic.showIcon(IconNames.Happy)

bluetooth.onBluetoothConnected(function () {
    verbonden = true
    basic.showIcon(IconNames.Yes)
})

bluetooth.onBluetoothDisconnected(function () {
    verbonden = false
    motors(0, 0)
    basic.showIcon(IconNames.No)
})

bluetooth.onUartDataReceived(serial.delimiters(Delimiters.NewLine), function () {
    let regel = bluetooth.uartReadUntil(serial.delimiters(Delimiters.NewLine))
    let delen = regel.split(" ")
    if (delen.length == 3 && delen[0] == "M") {
        if (!noodstop) {
            motors(parseInt(delen[1]), parseInt(delen[2]))
        }
        laatsteCmd = input.runningTime()
    } else if (delen[0] == "R") {
        noodstop = false
        basic.showIcon(IconNames.Yes)
    }
})

input.onButtonPressed(Button.A, function () {
    noodstop = true
    motors(0, 0)
    basic.showIcon(IconNames.Square)
    if (verbonden) {
        bluetooth.uartWriteLine("K A")
    }
})

input.onButtonPressed(Button.B, function () {
    noodstop = false
    basic.showIcon(IconNames.Yes)
    if (verbonden) {
        bluetooth.uartWriteLine("K B")
    }
})

input.onButtonPressed(Button.AB, function () {
    noodstop = false
    basic.showIcon(IconNames.House)
    if (verbonden) {
        bluetooth.uartWriteLine("K AB")
    }
})

basic.forever(function () {
    // Watchdog
    if (input.runningTime() - laatsteCmd > 500) {
        motors(0, 0)
    }
    // Sensoren doorsturen
    if (verbonden) {
        bluetooth.uartWriteLine("S " + pins.digitalReadPin(DigitalPin.P13) + " "
            + pins.digitalReadPin(DigitalPin.P14) + " " + afstandCm() + " "
            + input.compassHeading())
    }
    basic.pause(100)
})
