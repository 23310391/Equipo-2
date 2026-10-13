from machine import Pin, PWM
import time

# ---------- Pines ----------
PIN_RPWM, PIN_LPWM = 25, 26
PIN_R_EN, PIN_L_EN = 27, 14
PIN_SENSOR = 18

# ---------- Parámetros ----------
FREC_PWM = 20000
POTENCIA = 30             # % de PWM durante el movimiento
GRADOS_POR_SEGUNDO = 260
T_INERCIA = 0.0         # s que el motor sigue girando al cortar

# Sentido de giro de los movimientos: -1 invierte el giro físico
SENTIDO_GIRO = -1
SENTIDO_BUSQUEDA = -1

# Sensor: valor que entrega cuando DETECTA la marca (muchos módulos dan 0)
DETECTA_EN = 0
POTENCIA_BUSQUEDA = 25      # % durante la búsqueda del cero (más lento = más repetible)

# ---------- Hardware ----------
Pin(PIN_R_EN, Pin.OUT).value(1)
Pin(PIN_L_EN, Pin.OUT).value(1)
rpwm = PWM(Pin(PIN_RPWM), freq=FREC_PWM, duty_u16=0)
lpwm = PWM(Pin(PIN_LPWM), freq=FREC_PWM, duty_u16=0)
sensor = Pin(PIN_SENSOR, Pin.IN)

def mover_motor(potencia):
    """potencia > 0 = sentido positivo de los grados (ya con SENTIDO_GIRO aplicado)"""
    potencia = max(-100, min(100, potencia)) * SENTIDO_GIRO
    duty = int(abs(potencia) * 65535 / 100)
    if potencia >= 0:
        lpwm.duty_u16(0)
        rpwm.duty_u16(duty)
    else:
        rpwm.duty_u16(0)
        lpwm.duty_u16(duty)

def sensor_detecta():
    """Lectura con filtro: exige 3 lecturas seguidas iguales"""
    for _ in range(3):
        if sensor.value() != DETECTA_EN:
            return False
        time.sleep_ms(1)
    return True

def buscar_cero(tiempo_max=6):
    """Gira en SENTIDO_BUSQUEDA hasta detectar la marca. Devuelve True si la encontró."""
    mover_motor(SENTIDO_BUSQUEDA * POTENCIA_BUSQUEDA)
    t_inicio = time.ticks_ms()

    # Si arrancó sobre la marca, primero sale de ella para dar la vuelta completa
    while sensor_detecta():
        if time.ticks_diff(time.ticks_ms(), t_inicio) > tiempo_max * 1000:
            mover_motor(0)
            return False
        time.sleep_ms(2)

    # Gira hasta que la marca aparezca
    while not sensor_detecta():
        if time.ticks_diff(time.ticks_ms(), t_inicio) > tiempo_max * 1000:
            mover_motor(0)
            return False
        time.sleep_ms(1)

    mover_motor(0)
    time.sleep_ms(300)      # deja que se asiente la inercia
    return True

posicion_estimada = 0

def ir_a_posicion(objetivo):
    global posicion_estimada
    delta = objetivo - posicion_estimada
    if delta == 0:
        return
    tiempo = abs(delta) / GRADOS_POR_SEGUNDO - T_INERCIA
    tiempo = max(0, tiempo)
    signo = 1 if delta > 0 else -1

    mover_motor(signo * POTENCIA)
    time.sleep_ms(int(tiempo * 1000))
    mover_motor(0)

    posicion_estimada = objetivo

def buscar_y_fijar_cero():
    global posicion_estimada
    print("Buscando el cero con el sensor...")
    if buscar_cero():
        posicion_estimada = 0
        print("Cero encontrado")
        return True
    print("No se detectó la marca. Revisa el sensor, la marca y DETECTA_EN.")
    return False

# ---------- Programa principal ----------
try:
    if buscar_y_fijar_cero():
        print("Escribe los grados y presiona Enter.")
        print("  c = volver a buscar el cero    q = salir")
        while True:
            texto = input("Posición actual " + str(posicion_estimada) + "° -> ir a: ").strip().lower()

            if texto in ("q", "salir"):
                break
            if texto in ("c", "cero"):
                buscar_y_fijar_cero()
                continue

            try:
                objetivo = float(texto.replace(",", "."))
            except ValueError:
                print("Escribe un número (por ejemplo 90 o -45), 'c' o 'q'.")
                continue

            print("Yendo a", objetivo, "grados")
            ir_a_posicion(objetivo)
            print("Listo")
finally:
    mover_motor(0)
    rpwm.deinit()
    lpwm.deinit()