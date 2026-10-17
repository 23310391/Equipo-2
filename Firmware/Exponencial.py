from machine import Pin, PWM, disable_irq, enable_irq
from math import exp
import time

# ---------- Pines ----------
PIN_RPWM, PIN_LPWM = 25, 26
PIN_R_EN, PIN_L_EN = 27, 14
PIN_ENC_A, PIN_ENC_B = 32, 33   # los dos canales del encoder

# ---------- Exponencial ----------
TIPO = "carga"                  # "carga", "descarga" o "crecimiento"
RPM_PICO = 170                  # RPM máximas de la curva
TAU = 5.0                       # constante de tiempo en segundos (menor = más rápida)
DURACION = 20                  # segundos que dura una curva completa
REPETICIONES = 1                # cuántas veces se repite la curva
SENTIDO_GIRO = 1                # -1 invierte el sentido de giro

# ---------- Control sin PID ----------
RPM_MAX_ESTIMADA = 170          # RPM al 100 % de PWM
AJUSTE = 1.0                    # % que cambia la corrección por periodo
TOLERANCIA_RPM = 3
LIMITE_CORRECCION = 20          # % máximo de corrección
RPM_ZONA_MUERTA = 8             # por debajo de estas RPM apaga el motor

# ---------- Encoder ----------
PULSOS_POR_VUELTA = 370
MIN_PULSO_US = 150              # ignora pulsos separados menos de 150 us (ruido)
SENTIDO_ENCODER = 1             # pon -1 si las RPM salen con el signo contrario

# ---------- Otros parámetros ----------
FREC_PWM = 20000
TS_MS = 100                     # periodo de muestreo

# ---------- Hardware ----------
Pin(PIN_R_EN, Pin.OUT).value(1)
Pin(PIN_L_EN, Pin.OUT).value(1)
rpwm = PWM(Pin(PIN_RPWM), freq=FREC_PWM, duty_u16=0)
lpwm = PWM(Pin(PIN_LPWM), freq=FREC_PWM, duty_u16=0)

enc_a = Pin(PIN_ENC_A, Pin.IN, Pin.PULL_UP)
enc_b = Pin(PIN_ENC_B, Pin.IN, Pin.PULL_UP)

contador = 0
ultimo_pulso_us = 0

def contar_pulso(pin):
    global contador, ultimo_pulso_us
    ahora = time.ticks_us()
    if time.ticks_diff(ahora, ultimo_pulso_us) < MIN_PULSO_US:
        return
    ultimo_pulso_us = ahora
    if enc_b.value():
        contador -= SENTIDO_ENCODER
    else:
        contador += SENTIDO_ENCODER

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

def rpm_exponencial(tc):
    """RPM objetivo según el tipo de exponencial. tc = segundos dentro de la curva."""
    if TIPO == "carga":                 # sube y se aplana hacia RPM_PICO
        return RPM_PICO * (1 - exp(-tc / TAU))
    if TIPO == "descarga":              # arranca en RPM_PICO y decae hacia 0
        return RPM_PICO * exp(-tc / TAU)
    # "crecimiento": sube cada vez más rápido y llega a RPM_PICO al final
    return RPM_PICO * (exp(tc / TAU) - 1) / (exp(DURACION / TAU) - 1)

# ---------- Programa principal ----------
try:
    duracion_total = REPETICIONES * DURACION

    correccion = 0
    #rpm = 0.0
    cuenta_anterior = leer_contador()
    t_inicio = time.ticks_ms()
    t_anterior = t_inicio
    print("Iniciando exponencial (%s)..." % TIPO)

    while True:
        t = time.ticks_diff(time.ticks_ms(), t_inicio) / 1000
        if t >= duracion_total:
            break

        rpm_objetivo = rpm_exponencial(t % DURACION)

        if rpm_objetivo < RPM_ZONA_MUERTA:
            correccion = 0
            potencia = 0
        else:
            error = rpm_objetivo - rpm
            if error > TOLERANCIA_RPM:
                correccion += AJUSTE
            elif error < -TOLERANCIA_RPM:
                correccion -= AJUSTE
            correccion = max(-LIMITE_CORRECCION, min(LIMITE_CORRECCION, correccion))
            potencia = rpm_objetivo * 100 / RPM_MAX_ESTIMADA + correccion
            potencia = max(0, min(100, potencia))

        mover_motor(SENTIDO_GIRO * potencia)

        time.sleep_ms(TS_MS)

        # RPM medidas con los dos canales (positivas si gira en el sentido pedido)
        ahora = time.ticks_ms()
        dt = time.ticks_diff(ahora, t_anterior) / 1000
        t_anterior = ahora
        cuenta = leer_contador()
        rpm = SENTIDO_GIRO * (cuenta - cuenta_anterior) / PULSOS_POR_VUELTA / dt * 60
        cuenta_anterior = cuenta

        print("objetivo:%.1f,rpm:%.1f" % (rpm_objetivo, rpm))

    mover_motor(0)
    print("Fin")
finally:
    mover_motor(0)
    rpwm.deinit()
    lpwm.deinit()
