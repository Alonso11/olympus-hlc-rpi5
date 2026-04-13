// Version: v1.6
use pyo3::prelude::*;
use serialport;
use std::time::{Duration, Instant};
use std::io::{Read, Write};
use std::sync::Mutex;
use rppal::gpio::{Gpio, InputPin, OutputPin};

// ─── Nota de arquitectura — sensor ultrasónico HC-SR04 ────────────────────────
//
// El HC-SR04 del Rover Olympus está conectado físicamente al Arduino Mega
// (D38=Trigger, D39=Echo). La capa de emergencia (<20 cm → FAULT) se ejecuta
// en el firmware LLC (main.rs) cada 5 ciclos (~100 ms), y se comunica a la RPi5
// a través del protocolo MSM serie (respuesta TLM o transición a FAULT).
//
// Los campos `ultrasonic_trigger` / `ultrasonic_echo` y los métodos
// `setup_ultrasonic` / `get_ultrasonic_distance` de esta clase están pensados
// para un escenario futuro en el que se conecte un segundo HC-SR04 directamente
// a los GPIO de la RPi5 (ej. capa táctica de medio alcance independiente del LLC).
// Mientras ese hardware no esté presente, estos métodos no deben llamarse.
//
// ──────────────────────────────────────────────────────────────────────────────

#[pyclass]
struct Rover {
    port: Mutex<Box<dyn serialport::SerialPort>>,
    // Pines GPIO RPi5 para HC-SR04 secundario (futuro — ver nota de arquitectura)
    ultrasonic_trigger: Mutex<Option<OutputPin>>,
    ultrasonic_echo: Mutex<Option<InputPin>>,
}

#[pymethods]
impl Rover {
    #[new]
    #[pyo3(signature = (port_name="/dev/arduino_mega", baud_rate=115200))]
    fn new(port_name: &str, baud_rate: u32) -> PyResult<Self> {
        let port = serialport::new(port_name, baud_rate)
            .timeout(Duration::from_millis(100))
            .open()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyIOError, _>(format!("Error serial al abrir {}: {}", port_name, e)))?;

        // Al abrir el puerto, la mayoría de Arduinos se resetean (DTR).
        // Esperamos 2 segundos para que el bootloader termine y el firmware inicie.
        std::thread::sleep(Duration::from_secs(2));

        Ok(Rover {
            port: Mutex::new(port),
            ultrasonic_trigger: Mutex::new(None),
            ultrasonic_echo: Mutex::new(None),
        })
    }

    /// [FUTURO] Configura un HC-SR04 conectado directamente a los GPIO de la RPi5.
    /// NOTA: El HC-SR04 activo del rover está en el Arduino (D38/D39).
    ///       Este método es para un segundo sensor en GPIO RPi5 (no instalado aún).
    fn setup_ultrasonic(&self, trigger_pin: u8, echo_pin: u8) -> PyResult<String> {
        let gpio = Gpio::new()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(format!("Error GPIO: {}", e)))?;

        let mut trigger = gpio.get(trigger_pin)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(format!("Error Pin Trigger {}: {}", trigger_pin, e)))?
            .into_output();
        
        let echo = gpio.get(echo_pin)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>(format!("Error Pin Echo {}: {}", echo_pin, e)))?
            .into_input();

        // Aseguramos que el trigger inicie en bajo
        trigger.set_low();

        let mut t_lock = self.ultrasonic_trigger.lock().unwrap();
        let mut e_lock = self.ultrasonic_echo.lock().unwrap();
        
        *t_lock = Some(trigger);
        *e_lock = Some(echo);

        Ok(format!("Ultrasonico configurado: Trig={}, Echo={}", trigger_pin, echo_pin))
    }

    /// [FUTURO] Mide la distancia en mm desde el HC-SR04 en GPIO RPi5.
    /// NOTA: Requiere llamar setup_ultrasonic primero. No aplica al sensor del Arduino.
    fn get_ultrasonic_distance(&self) -> PyResult<Option<f64>> {
        let mut t_lock = self.ultrasonic_trigger.lock().unwrap();
        let e_lock = self.ultrasonic_echo.lock().unwrap();

        let (trigger, echo) = match (t_lock.as_mut(), e_lock.as_ref()) {
            (Some(t), Some(e)) => (t, e),
            _ => return Err(PyErr::new::<pyo3::exceptions::PyRuntimeError, _>("Sensor ultrasonico no configurado. Llame a setup_ultrasonic primero.")),
        };

        // 1. Enviar pulso de disparo (10 microsegundos)
        trigger.set_high();
        std::thread::sleep(Duration::from_micros(10));
        trigger.set_low();

        // 2. Esperar a que el Echo suba (inicio del pulso)
        let start_wait = Instant::now();
        while echo.is_low() {
            if start_wait.elapsed() > Duration::from_millis(30) { return Ok(None); }
        }
        let pulse_start = Instant::now();

        // 3. Esperar a que el Echo baje (fin del pulso)
        while echo.is_high() {
            if pulse_start.elapsed() > Duration::from_millis(30) { return Ok(None); }
        }
        let pulse_duration = pulse_start.elapsed();

        // 4. Calcular distancia: (tiempo * velocidad_sonido) / 2
        // Velocidad del sonido aprox 343 m/s = 0.343 mm/us
        let distance_mm = (pulse_duration.as_micros() as f64 * 0.343) / 2.0;

        if distance_mm > 4000.0 || distance_mm < 20.0 {
            Ok(None)
        } else {
            Ok(Some(distance_mm))
        }
    }

    /// Lee hasta una línea del puerto sin enviar comando (50 ms de timeout).
    /// Retorna el frame si empieza por "TLM:", None en cualquier otro caso.
    ///
    /// Usar al inicio de cada ciclo del loop para drenar frames TLM asíncronos
    /// emitidos por el firmware (~1 s). No bloquea el loop principal.
    fn recv_tlm(&self) -> PyResult<Option<String>> {
        let mut port = self.port.lock()
            .map_err(|_| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>("Mutex error en puerto serie"))?;

        let mut line = Vec::with_capacity(160);
        let mut buf  = [0u8; 1];
        let deadline = Instant::now() + Duration::from_millis(50);

        loop {
            if Instant::now() >= deadline {
                return Ok(None);
            }
            match port.read(&mut buf) {
                Ok(1) => {
                    if buf[0] == b'\n' {
                        if line.starts_with(b"TLM:") {
                            let s = String::from_utf8_lossy(&line)
                                .trim_end_matches('\r')
                                .to_string();
                            return Ok(Some(s));
                        }
                        // Llegó otra cosa (ACK rezagado, etc.) — descartar
                        return Ok(None);
                    }
                    line.push(buf[0]);
                }
                Ok(_) => continue,
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => return Ok(None),
                Err(e) => return Err(PyErr::new::<pyo3::exceptions::PyIOError, _>(
                    format!("Error lectura recv_tlm: {}", e)
                )),
            }
        }
    }

    /// Envía un comando MSM al Arduino y retorna la respuesta ASCII.
    /// Protocolo: envía "<cmd>\n", lee hasta '\n' (timeout 300 ms).
    /// Respuestas esperadas: PONG, ACK:<STATE>, TLM:<SAFETY>:<MASK>, ERR:*
    ///
    /// Los frames TLM: llegan asincrónicamente cada ~1 s desde el firmware.
    /// Si un TLM llega antes que el ACK al comando enviado, se descarta y
    /// se sigue leyendo hasta obtener la respuesta real (o timeout).
    fn send_command(&self, cmd: String) -> PyResult<String> {
        let mut port = self.port.lock()
            .map_err(|_| PyErr::new::<pyo3::exceptions::PyRuntimeError, _>("Mutex error en puerto serie"))?;

        // 1. Enviar comando con terminador de línea
        let formatted_cmd = format!("{}\n", cmd);
        port.write_all(formatted_cmd.as_bytes())
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyIOError, _>(format!("Error escritura: {}", e)))?;
        port.flush()
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyIOError, _>(format!("Error flush: {}", e)))?;

        // 2. Leer respuesta byte a byte hasta '\n' o timeout de 300 ms.
        //    Si el frame leído empieza por "TLM:" es telemetría asincrónica —
        //    se descarta y se sigue leyendo dentro del mismo timeout.
        let mut response = Vec::with_capacity(24);
        let mut buf = [0u8; 1];
        let deadline = Instant::now() + Duration::from_millis(300);

        'outer: loop {
            if Instant::now() >= deadline {
                return Err(PyErr::new::<pyo3::exceptions::PyTimeoutError, _>(
                    format!("Timeout esperando respuesta a '{}'", cmd)
                ));
            }
            match port.read(&mut buf) {
                Ok(1) => {
                    if buf[0] == b'\n' {
                        // Frame completo — descartar si no es una respuesta de protocolo.
                        // Prefijos válidos: PONG, ACK:, ERR:
                        // Prefijos async a ignorar: TLM:, WARN:, INFO:, RESET:, DBG:, RAW:
                        let is_protocol = response.starts_with(b"PONG")
                            || response.starts_with(b"ACK:")
                            || response.starts_with(b"ERR:");
                        if !is_protocol {
                            response.clear();
                            continue 'outer;
                        }
                        break 'outer;
                    }
                    response.push(buf[0]);
                }
                Ok(_) => continue,
                Err(ref e) if e.kind() == std::io::ErrorKind::TimedOut => continue,
                Err(e) => return Err(PyErr::new::<pyo3::exceptions::PyIOError, _>(
                    format!("Error lectura: {}", e)
                )),
            }
        }

        // 3. Convertir a String (quitar posible '\r')
        let resp_str = String::from_utf8_lossy(&response)
            .trim_end_matches('\r')
            .to_string();

        Ok(resp_str)
    }
}

#[pymodule]
fn rover_bridge(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<Rover>()?;
    Ok(())
}
