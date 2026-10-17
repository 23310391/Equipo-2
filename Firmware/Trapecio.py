from machine import Pin, PWM, disable_irq, enable_irq
from math import sin, pi
import time

# ---------- Pines ----------
PIN_RPWM, PIN_LPWM = 25, 26
PIN_R_EN, PIN_L_EN = 27, 14
PIN_ENC_A, PIN_ENC_B = 32, 33

# ---------- Parámetros ----------
PULSOS_POR_VUELTA = 274
FREC_PWM = 20000
TS_MS = 100                 # periodo de muestreo

# ---------- Onda senoidal ----------
RPM_CENTRO = 85             # valor medio de la onda
AMPLITUD = 70               # RPM arriba y abajo del centro (rango: 15 a 155)
PERIODO = 10                # segundos por ciclo
CICLOS = 3                  # cuántos ciclos completos hace

# ---------- Control sin PID ----------
RPM_MAX_ESTIMADA = 170      # RPM que da el motor al 100 %
AJUSTE_AUTOMATICO = True    # False = lazo abierto puro (senoidal de potencia)
AJUSTE = 1.0                # % que cambia la corrección por periodo
TOLERANCIA_RPM = 3          # no corrige si está dentro de ±3 RPM
LIMITE_CORRECCION = 20      # % máximo que puede sumar o restar la corrección

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

# ---------- Programa principal ----------
try:
    correccion = 0
    cuenta_anterior = leer_contador()
    t_inicio = time.ticks_ms()
    t_anterior = t_inicio
    duracion_total = CICLOS * PERIODO

    while True:
        time.sleep_ms(TS_MS)
        ahora = time.ticks_ms()
        t = time.ticks_diff(ahora, t_inicio) / 1000
        if t >= duracion_total:
            break

        # Medición de RPM
        dt = time.ticks_diff(ahora, t_anterior) / 1000
        t_anterior = ahora
        cuenta = leer_contador()
        rpm = (cuenta - cuenta_anterior) / PULSOS_POR_VUELTA / dt * 60
        cuenta_anterior = cuenta

        # RPM objetivo: senoidal que arranca en su valor mínimo (fase -90°)
        rpm_objetivo = RPM_CENTRO + AMPLITUD * sin(2 * pi * t / PERIODO - pi / 2)

        # Ajuste por escalones sobre la potencia estimada
        if AJUSTE_AUTOMATICO:
            error = rpm_objetivo - rpm
            if error > TOLERANCIA_RPM:
                correccion += AJUSTE
            elif error < -TOLERANCIA_RPM:
                correccion -= AJUSTE
            correccion = max(-LIMITE_CORRECCION, min(LIMITE_CORRECCION, correccion))

        potencia = rpm_objetivo * 100 / RPM_MAX_ESTIMADA + correccion
        mover_motor(potencia)

        # formato compatible con el Plotter de Thonny
        print(round(rpm_objetivo, 1), round(rpm, 1))
finally:
    mover_motor(0)
    rpwm.deinit()
    lpwm.deinit()
