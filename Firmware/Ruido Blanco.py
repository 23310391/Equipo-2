from machine import Pin, PWM, disable_irq, enable_irq
import time, random

# ---------- Pines ----------
PIN_RPWM, PIN_LPWM = 25, 26
PIN_R_EN, PIN_L_EN = 27, 14
PIN_ENC_A, PIN_ENC_B = 32, 33

# ---------- Parámetros ----------
PULSOS_POR_VUELTA = 600
FREC_PWM = 20000
TS_MS = 100             # periodo de muestreo
POT_MIN = 30            # % mínimo (arriba de la zona muerta del motor)
POT_MAX = 100           # % máximo
CAMBIA_CADA = 1         # periodos antes de generar un nuevo valor aleatorio
DURACION_S = 60         # duración de la prueba

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
    """potencia en % de 0 a 100 (solo sentido positivo)"""
    potencia = max(0, min(100, potencia))
    duty = int(potencia * 65535 / 100)
    lpwm.duty_u16(0)
    rpwm.duty_u16(duty)

# ---------- Programa principal ----------
print("Ruido blanco iniciado. Ctrl+C para detener.")
print("tiempo_s,potencia,rpm")

potencia = 0
cuenta_anterior = leer_contador()
t_inicio = time.ticks_ms()
t_anterior = t_inicio
ciclo = 0

try:
    while time.ticks_diff(time.ticks_ms(), t_inicio) < DURACION_S * 1000:
        # Nueva potencia aleatoria
        if ciclo % CAMBIA_CADA == 0:
            potencia = random.randint(POT_MIN, POT_MAX)
        mover_motor(potencia)
        ciclo += 1

        time.sleep_ms(TS_MS)

        # Cálculo de RPM
        ahora = time.ticks_ms()
        dt = time.ticks_diff(ahora, t_anterior) / 1000
        t_anterior = ahora
        cuenta = leer_contador()
        rpm = (cuenta - cuenta_anterior) / PULSOS_POR_VUELTA / dt * 60
        cuenta_anterior = cuenta

        t = time.ticks_diff(ahora, t_inicio) / 1000
        print("{:.2f},{},{:.1f}".format(t, potencia, rpm))
finally:
    mover_motor(0)
    rpwm.deinit()
    lpwm.deinit()
    print("Prueba terminada.")
