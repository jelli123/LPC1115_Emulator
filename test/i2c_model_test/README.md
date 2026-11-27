# I2C-Modell-Test (Host)

Prüft die I2C-Master-Zustandsmaschine aus `src/peripherals.cpp` auf dem PC.
`run.sh` schneidet `i2c_reset_txn()` … `i2c_write_byte()` aus der Quelle aus,
setzt sie mit Stubs für Bus-Zugriff und Interrupt zusammen und treibt sie mit
der Logik von `handleMasterXferState()` der sblib (Polling-Modus, wartet ohne
Timeout auf SI) gegen einen simulierten Slave (Adresse 0x44, wie SHT4x).

```
test/i2c_model_test/run.sh
```

Abgedeckt: Senden, Lesen, NAK während der Messung mit anschließenden Daten,
Schreiben + Repeated START + Lesen, dasselbe ohne Slave und ohne Bridge
(kein Hängen, NAK bzw. 0 Bytes), Ruhezustand 0xF8 am Ende.

Ändert sich der Aufbau der I2C-Funktionen in `peripherals.cpp`, müssen die
Stubs in `host_test.cpp` (`i2c_flush_write`, `i2c_hw_read_byte`,
`i2c_hw_release`, `I2cModel`) angepasst werden.
