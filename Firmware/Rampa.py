from machine import Pin, PWM, disable_irq, enable_irq
import time

# ---------- Pines ----------
PIN_RPWM, PIN_LPWM = 25, 26
PIN_R_EN, PIN_L_EN = 27, 14
PIN_ENC_A, PIN_ENC_B = 32, 33

# ---------- Parámetros ----------
PULSOS_POR_VUELTA = 370
FREC_PWM = 20000
TS_MS = 100                 # periodo de muestreo

# ---------- Rampa de RPM ----------
RPM_INICIAL = 0
RPM_FINAL = 170
PASO_RPM = 10
TIEMPO_ESCALON = 0.5         # segundos que se mantiene cada escalón

# ---------- Control sin PID ----------
RPM_MAX_ESTIMADA = 170      # RPM que da el motor al 100 % (para la potencia estimada)
AJUSTE_AUTOMATICO = False    # False = lazo abierto puro (solo potencia estimada)
AJUSTE = 8.0                # % que sube o baja la potencia por periodo
TOLERANCIA_RPM = 3          # no corrige si está dentro de ±3 RPM del objetivo

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

# ---------- Estado ----------
potencia = 0
rpm_objetivo_anterior = 0
cuenta_anterior = 0
t_anterior = 0

def mantener_rpm(rpm_objetivo, duracion):
    """Sube la potencia estimada al nuevo escalón y la ajusta de 1 % en 1 %."""
    global potencia, rpm_objetivo_anterior, cuenta_anterior, t_anterior

    # Potencia estimada para este escalón (parte de la del escalón anterior)
    potencia += (rpm_objetivo - rpm_objetivo_anterior) * 100 / RPM_MAX_ESTIMADA
    potencia = max(0, min(100, potencia))
    if rpm_objetivo == 0:
        potencia = 0
    rpm_objetivo_anterior = rpm_objetivo

    t_fin = time.ticks_add(time.ticks_ms(), int(duracion * 1000))
    while time.ticks_diff(t_fin, time.ticks_ms()) > 0:
        mover_motor(potencia)
        time.sleep_ms(TS_MS)

        # Medición de RPM
        ahora = time.ticks_ms()
        dt = time.ticks_diff(ahora, t_anterior) / 1000
        t_anterior = ahora
        cuenta = leer_contador()
        rpm = (cuenta - cuenta_anterior) / PULSOS_POR_VUELTA / dt * 60
        cuenta_anterior = cuenta

        # Ajuste por escalones
        if AJUSTE_AUTOMATICO and rpm_objetivo > 0:
            if rpm < rpm_objetivo - TOLERANCIA_RPM:
                potencia += AJUSTE
            elif rpm > rpm_objetivo + TOLERANCIA_RPM:
                potencia -= AJUSTE
            potencia = max(0, min(100, potencia))

        # formato compatible con el Plotter de Thonny
        print(rpm_objetivo, round(rpm, 1))

# ---------- Programa principal ----------
try:
    cuenta_anterior = leer_contador()
    t_anterior = time.ticks_ms()

    for rpm_objetivo in range(RPM_INICIAL, RPM_FINAL + 1, PASO_RPM):
        mantener_rpm(rpm_objetivo, TIEMPO_ESCALON)

    mantener_rpm(0, 2)                           # baja a 0 al terminar
finally:
    mover_motor(0)
    rpwm.deinit()
    lpwm.deinit()
