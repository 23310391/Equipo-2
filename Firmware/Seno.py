from machine import Pin, PWM, ADC, disable_irq, enable_irq
from math import sin, pi
import time

# ---------- Pines ----------
PIN_RPWM, PIN_LPWM = 25, 26
PIN_R_EN, PIN_L_EN = 27, 14
PIN_ENC_A = 32                  # solo canal A
PIN_IS_R, PIN_IS_L = 34, 35     # sensado de corriente del BTS7960
PIN_SENSOR = 18                 # sensor infrarrojo (posición 0)

# ---------- Encoder ----------
PULSOS_POR_VUELTA = 370
MIN_PULSO_US = 150              # ignora pulsos separados menos de 150 us (ruido)

# ---------- Onda senoidal ----------
RPM_PICO = 100                  # RPM máximas (en cada sentido)
PERIODO = 10                    # segundos por ciclo completo
CICLOS = 2                      # cuántos ciclos hace
SENTIDO_GIRO = 1                # -1 invierte cuál sentido es el "positivo"
RPM_ZONA_MUERTA = 8             # por debajo de estas RPM apaga el motor (cruce por cero)

# ---------- Control sin PID ----------
FREC_PWM = 20000
TS_MS = 100                     # periodo de muestreo
RPM_MAX_ESTIMADA = 170          # RPM al 100 % de PWM
AJUSTE = 1.0                    # % que cambia la corrección por periodo
TOLERANCIA_RPM = 3
LIMITE_CORRECCION = 20          # % máximo de corrección

# ---------- Sensado de corriente ----------
R_IS = 4700                     # ohmios de la resistencia IS a GND que pusiste
K_ILIS = 8500
MUESTRAS_ADC = 40
DUTY_MINIMO = 0.05

# ---------- Posición 0 con sensor ----------
USAR_SENSOR_CERO = True
SENTIDO_BUSQUEDA = 1            # 1 = potencia positiva cruda (RPWM), -1 = contraria
POTENCIA_BUSQUEDA = 25
DETECTA_EN = 0                  # valor que da el sensor cuando detecta la marca

# ---------- Hardware ----------
Pin(PIN_R_EN, Pin.OUT).value(1)
Pin(PIN_L_EN, Pin.OUT).value(1)
rpwm = PWM(Pin(PIN_RPWM), freq=FREC_PWM, duty_u16=0)
lpwm = PWM(Pin(PIN_LPWM), freq=FREC_PWM, duty_u16=0)
sensor = Pin(PIN_SENSOR, Pin.IN)

enc_a = Pin(PIN_ENC_A, Pin.IN, Pin.PULL_UP)

adc_r = ADC(Pin(PIN_IS_R))
adc_l = ADC(Pin(PIN_IS_L))
adc_r.atten(ADC.ATTN_0DB)
adc_l.atten(ADC.ATTN_0DB)

contador = 0
ultimo_pulso_us = 0

def contar_pulso(pin):
    global contador, ultimo_pulso_us
    ahora = time.ticks_us()
    if time.ticks_diff(ahora, ultimo_pulso_us) < MIN_PULSO_US:
        return
    ultimo_pulso_us = ahora
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

# ---------- Lectura de corriente ----------
def leer_voltaje(adc):
    total = 0
    for _ in range(MUESTRAS_ADC):
        try:
            total += adc.read_uv() / 1_000_000
        except AttributeError:
            total += adc.read() * 1.1 / 4095
        time.sleep_us(137)
    return total / MUESTRAS_ADC

time.sleep_ms(300)
offset_r = leer_voltaje(adc_r)
offset_l = leer_voltaje(adc_l)
if offset_r > 0.3 or offset_l > 0.3:
    print("AVISO: IS marca %.2f V / %.2f V con el motor parado. Revisa la resistencia IS a GND."
          % (offset_r, offset_l))

def leer_corriente(potencia):
    duty = abs(potencia) / 100
    if duty < DUTY_MINIMO:
        return 0.0
    v_r = max(0, leer_voltaje(adc_r) - offset_r)
    v_l = max(0, leer_voltaje(adc_l) - offset_l)
    return (v_r + v_l) / R_IS * K_ILIS / duty

# ---------- Búsqueda del cero con el sensor ----------
def sensor_detecta():
    for _ in range(3):
        if sensor.value() != DETECTA_EN:
            return False
        time.sleep_ms(1)
    return True

def buscar_cero(tiempo_max=6):
    mover_motor(SENTIDO_BUSQUEDA * POTENCIA_BUSQUEDA)
    t_inicio = time.ticks_ms()

    while sensor_detecta():                 # si arrancó sobre la marca, sale de ella
        if time.ticks_diff(time.ticks_ms(), t_inicio) > tiempo_max * 1000:
            mover_motor(0)
            return False
        time.sleep_ms(2)

    while not sensor_detecta():             # gira hasta que aparezca la marca
        if time.ticks_diff(time.ticks_ms(), t_inicio) > tiempo_max * 1000:
            mover_motor(0)
            return False
        time.sleep_ms(1)

    mover_motor(0)
    time.sleep_ms(300)
    return True

# ---------- Programa principal ----------
try:
    if USAR_SENSOR_CERO:
        print("Buscando el cero con el sensor...")
        if not buscar_cero():
            print("No se detectó la marca. Revisa el sensor, la marca y DETECTA_EN.")
            raise SystemExit
        print("Cero encontrado")
        time.sleep(1)

    posicion = 0.0              # grados respecto al cero
    correccion = 0
    rpm = 0.0
    cuenta_anterior = leer_contador()
    t_inicio = time.ticks_ms()
    t_anterior = t_inicio
    duracion_total = CICLOS * PERIODO
    print("Iniciando senoidal...")

    while True:
        t = time.ticks_diff(time.ticks_ms(), t_inicio) / 1000
        if t >= duracion_total:
            break

        # RPM objetivo: seno con signo (positivo = un sentido, negativo = el otro)
        rpm_objetivo = RPM_PICO * sin(2 * pi * t / PERIODO)

        if abs(rpm_objetivo) < RPM_ZONA_MUERTA:
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
            potencia = max(-100, min(100, potencia))

        mover_motor(SENTIDO_GIRO * potencia)
        corriente = leer_corriente(potencia)

        time.sleep_ms(TS_MS)

        # RPM y posición (con solo el canal A, el signo lo da el semiciclo)
        ahora = time.ticks_ms()
        dt = time.ticks_diff(ahora, t_anterior) / 1000
        t_anterior = ahora
        cuenta = leer_contador()
        pulsos = cuenta - cuenta_anterior
        cuenta_anterior = cuenta
        signo = 1 if rpm_objetivo >= 0 else -1
        rpm = signo * pulsos / PULSOS_POR_VUELTA / dt * 60
        posicion += signo * pulsos / PULSOS_POR_VUELTA * 360

        print("objetivo:%.1f,rpm:%.1f,corriente:%.2f,posicion:%.1f"
              % (rpm_objetivo, rpm, corriente, posicion))

    mover_motor(0)
    print("Fin. Posición final estimada: %.1f grados" % posicion)
except SystemExit:
    pass
finally:
    mover_motor(0)
    rpwm.deinit()
    lpwm.deinit()
