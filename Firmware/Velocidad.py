from machine import Pin, PWM, disable_irq, enable_irq
import sys, select, time

# ---------- Pines ----------
PIN_RPWM, PIN_LPWM = 25, 26
PIN_R_EN, PIN_L_EN = 27, 14
PIN_ENC_A, PIN_ENC_B = 32, 33

# ---------- Parámetros ----------
PULSOS_POR_VUELTA = 600
FREC_PWM = 20000
TS_MS = 100             # periodo de actualización
PASO_RAMPA = 10         # % máximo que cambia la potencia por periodo

# ---------- Hardware ----------
Pin(PIN_R_EN, Pin.OUT).value(1)
Pin(PIN_L_EN, Pin.OUT).value(1)
rpwm = PWM(Pin(PIN_RPWM), freq=FREC_PWM, duty_u16=0)
lpwm = PWM(Pin(PIN_LPWM), freq=FREC_PWM, duty_u16=0)

enc_a = Pin(PIN_ENC_A, Pin.IN, Pin.PULL_UP)
enc_b = Pin(PIN_ENC_B, Pin.IN, Pin.PULL_UP)

contador = 0

def contar_pulso(pin):
    global contador
    if enc_b.value():
        contador -= 1
    else:
        contador += 1

enc_a.irq(trigger=Pin.IRQ_RISING, handler=contar_pulso)

def leer_contador():
    estado = disable_irq()
    valor = contador
    enable_irq(estado)
    return valor

def mover_motor(potencia):
    """potencia en % de -100 a 100 (el signo define el sentido)"""
    potencia = max(-100, min(100, potencia))
    duty = int(abs(potencia) * 65535 / 100)
    if potencia >= 0:
        lpwm.duty_u16(0)
        rpwm.duty_u16(duty)
    else:
        rpwm.duty_u16(0)
        lpwm.duty_u16(duty)

# ---------- Lectura no bloqueante de la terminal ----------
sondeo = select.poll()
sondeo.register(sys.stdin, select.POLLIN)

def leer_comando():
    if sondeo.poll(0):
        linea = sys.stdin.readline().strip()
        try:
            return float(linea)
        except ValueError:
            print("Escribe un número entre -100 y 100")
    return None

# ---------- Programa principal ----------
print("Escribe una potencia (-100 a 100) y presiona Enter. 0 = parar.")

potencia_objetivo = 0
potencia_actual = 0
cuenta_anterior = leer_contador()
t_anterior = time.ticks_ms()

try:
    while True:
        comando = leer_comando()
        if comando is not None:
            potencia_objetivo = max(-100, min(100, comando))

        # Rampa suave hacia la potencia pedida
        if potencia_actual < potencia_objetivo:
            potencia_actual = min(potencia_actual + PASO_RAMPA, potencia_objetivo)
        elif potencia_actual > potencia_objetivo:
            potencia_actual = max(potencia_actual - PASO_RAMPA, potencia_objetivo)
        mover_motor(potencia_actual)

        time.sleep_ms(TS_MS)

        # Cálculo de RPM
        ahora = time.ticks_ms()
        dt = time.ticks_diff(ahora, t_anterior) / 1000
        t_anterior = ahora
        cuenta = leer_contador()
        rpm = (cuenta - cuenta_anterior) / PULSOS_POR_VUELTA / dt * 60
        cuenta_anterior = cuenta

        print("potencia:", potencia_actual, "rpm:", round(rpm, 1))
finally:
    mover_motor(0)
    rpwm.deinit()
    lpwm.deinit()
