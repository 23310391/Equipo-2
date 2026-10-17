from machine import Pin, PWM, disable_irq, enable_irq
import time

# ---------- Pines ----------
PIN_RPWM, PIN_LPWM = 25, 26
PIN_R_EN, PIN_L_EN = 27, 14
PIN_ENC_A, PIN_ENC_B = 32, 33   # los dos canales del encoder

# ---------- Onda cuadrada ----------
POTENCIA_ALTA = 60              # % durante el nivel alto
POTENCIA_BAJA = 0               # % durante el nivel bajo (negativo = giro inverso)
T_ALTO = 2.0                    # segundos en nivel alto
T_BAJO = 2.0                    # segundos en nivel bajo
CICLOS = 5                      # cuántos pulsos completos hace
PAUSA_INVERSION_MS = 200        # pausa al cambiar de sentido (protege el driver)

# ---------- Encoder ----------
PULSOS_POR_VUELTA = 370         # pulsos del canal A por vuelta del eje
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
    # El canal B indica el sentido
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

# ---------- Programa principal ----------
try:
    periodo = T_ALTO + T_BAJO
    duracion_total = CICLOS * periodo

    potencia_anterior = 0
    cuenta_anterior = leer_contador()
    t_inicio = time.ticks_ms()
    t_anterior = t_inicio
    print("Iniciando pulsos...")

    while True:
        t = time.ticks_diff(time.ticks_ms(), t_inicio) / 1000
        if t >= duracion_total:
            break

        # Onda cuadrada: nivel alto al inicio de cada ciclo, luego nivel bajo
        fase = t % periodo
        potencia = POTENCIA_ALTA if fase < T_ALTO else POTENCIA_BAJA

        # Si cambia de sentido, detiene un instante antes de invertir
        if potencia * potencia_anterior < 0:
            mover_motor(0)
            time.sleep_ms(PAUSA_INVERSION_MS)
        potencia_anterior = potencia

        mover_motor(potencia)

        time.sleep_ms(TS_MS)

        # RPM con signo, medidas con los dos canales
        ahora = time.ticks_ms()
        dt = time.ticks_diff(ahora, t_anterior) / 1000
        t_anterior = ahora
        cuenta = leer_contador()
        rpm = (cuenta - cuenta_anterior) / PULSOS_POR_VUELTA / dt * 60
        cuenta_anterior = cuenta

        print("potencia:%.0f,rpm:%.1f" % (potencia, rpm))

    mover_motor(0)
    print("Fin")
finally:
    mover_motor(0)
    rpwm.deinit()
    lpwm.deinit()
