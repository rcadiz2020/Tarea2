#ifndef BME688_H
#define BME688_H

#include "esp_err.h"

typedef struct {
    float temperature;      // Grados Celsius
    float pressure;         // Pascales (Pa)
    float humidity;         // % Humedad Relativa
    float gas_resistance;   // Ohms
} bme688_data_t;

/**
 * @brief Inicializa I2C, lee coeficientes de calibración y configura el sensor.
 */
esp_err_t bme688_init(void);

/**
 * @brief Activa modo forzado, espera la medición y devuelve todos los datos compensados.
 */
esp_err_t bme688_read_data(bme688_data_t *data);

#endif // BME688_H